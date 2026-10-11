// Gate T1 (DualChannel.md): does AP-scheduled access actually beat CSMA contention, and by how
// much per hop? This is the number that decides whether the dual-channel rework is worth doing --
// the whole latency argument rests on "the AP schedules instead of everyone contending", and it
// has never been measured.
//
// Same chain, same spacing, same traffic, two MACs:
//   --mode=adhoc : today's harness stack, one channel, AdhocWifiMac, CSMA/CA for every hop
//   --mode=bss   : a chain of infrastructure BSSs, node i = AP toward i+1 and STA toward i-1,
//                  one channel per hop so no two hops interfere (the clean floor)
// Both use SpectrumWifiPhy so the PHY model is not a variable (Yans cannot do OFDMA at all).
//
// Each BSS here has exactly ONE STA, so this is SU-OFDMA: no RU splitting, the gain measured is
// purely "scheduled access instead of contention". MU only appears where a node has 2+ children.
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/wifi-module.h"

#include <iomanip>
#include <iostream>
#include <vector>

using namespace ns3;
using namespace std;

namespace {

// 80 MHz centre channels (20 MHz numbers like 36 abort: "No unique channel found").
const vector<uint32_t> CH80 = {42, 58, 106, 122, 138, 155, 171};

struct Result
{
    double delayMs = 0.0;
    double thptMbps = 0.0;
    double lossPct = 0.0;
    uint64_t rxPackets = 0;
};

Ptr<MultiModelSpectrumChannel>
MakeChannel()
{
    Ptr<MultiModelSpectrumChannel> c = CreateObject<MultiModelSpectrumChannel>();
    c->AddPropagationLossModel(CreateObject<LogDistancePropagationLossModel>());
    c->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    return c;
}

Result
Collect(Ptr<FlowMonitor> fm, FlowMonitorHelper& fmh, uint16_t port)
{
    Result r;
    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    for (const auto& [id, st] : fm->GetFlowStats())
    {
        if (cl->FindFlow(id).destinationPort != port)
        {
            continue;
        }
        double dur = (st.timeLastRxPacket - st.timeFirstRxPacket).GetSeconds();
        r.rxPackets = st.rxPackets;
        r.delayMs = (st.rxPackets > 0) ? st.delaySum.GetSeconds() / st.rxPackets * 1e3 : 0.0;
        r.thptMbps = (dur > 0) ? st.rxBytes * 8.0 / dur / 1e6 : 0.0;
        r.lossPct = (st.txPackets > 0) ? 100.0 * (st.txPackets - st.rxPackets) / st.txPackets : 0.0;
    }
    return r;
}

// ADHOC = today's harness (1 radio, 1 channel). ADHOC_MC = same radios/channels/subnets as BSS
// but an ad-hoc MAC, so BSS-vs-ADHOC_MC isolates *scheduling* from the extra spectrum.
enum class Mode
{
    ADHOC,
    ADHOC_MC,
    BSS
};

Result
RunChain(uint32_t hops, double spacing, const string& rate, Mode mode, uint32_t nChannels)
{
    const bool bss = (mode == Mode::BSS);
    const bool perHopChannel = (mode != Mode::ADHOC);
    const uint16_t PORT = 5001;
    const uint32_t N = hops + 1;

    NodeContainer nodes;
    nodes.Create(N);

    MobilityHelper mob;
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < N; ++i)
    {
        pos->Add(Vector(i * spacing, 0.0, 0.0));
    }
    mob.SetPositionAllocator(pos);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    InternetStackHelper internet;
    Ipv4StaticRoutingHelper srh;
    internet.SetRoutingHelper(srh);
    internet.Install(nodes);

    Ipv4AddressHelper addr;
    Ipv4Address dstAddr;
    // apIface[i] = interface index on node i facing node i+1 (BSS mode only).
    vector<uint32_t> apIface(N, 1);
    vector<Ipv4Address> nextHop(N);

    if (!perHopChannel)
    {
        // --- ad-hoc baseline: one channel, one flat subnet, CSMA/CA every hop ---
        SpectrumWifiPhyHelper phy;
        phy.SetChannel(MakeChannel());
        phy.Set("ChannelSettings", StringValue("{" + to_string(CH80[0]) + ", 80, BAND_5GHZ, 0}"));
        WifiMacHelper mac;
        mac.SetType("ns3::AdhocWifiMac");
        NetDeviceContainer dev = wifi.Install(phy, mac, nodes);

        addr.SetBase("10.1.1.0", "255.255.255.0");
        Ipv4InterfaceContainer ifc = addr.Assign(dev);
        dstAddr = ifc.GetAddress(N - 1);
        for (uint32_t i = 0; i + 1 < N; ++i)
        {
            nextHop[i] = ifc.GetAddress(i + 1);
        }
    }
    else
    {
        // --- BSS chain: hop i is its own infrastructure BSS on its own channel ---
        // node i runs an AP facing i+1 and a STA facing i-1; those two must differ in channel,
        // which giving every hop a distinct channel satisfies automatically.
        vector<Ptr<MultiModelSpectrumChannel>> chans(nChannels);
        for (uint32_t c = 0; c < nChannels; ++c)
        {
            chans[c] = MakeChannel();
        }
        vector<NetDeviceContainer> apDev(N);
        vector<NetDeviceContainer> staDev(N);

        for (uint32_t h = 0; h < hops; ++h)
        {
            uint32_t c = h % nChannels;
            SpectrumWifiPhyHelper phy;
            phy.SetChannel(chans[c]);
            phy.Set("ChannelSettings",
                    StringValue("{" + to_string(CH80[c % CH80.size()]) + ", 80, BAND_5GHZ, 0}"));
            if (bss)
            {
                Ssid ssid = Ssid("bss-" + to_string(h));
                WifiMacHelper macAp;
                macAp.SetMultiUserScheduler("ns3::RrMultiUserScheduler");
                macAp.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
                apDev[h] = wifi.Install(phy, macAp, nodes.Get(h));

                WifiMacHelper macSta;
                macSta.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
                staDev[h + 1] = wifi.Install(phy, macSta, nodes.Get(h + 1));
            }
            else
            {
                // The control: identical radios/channels/subnets, ad-hoc MAC on both ends.
                WifiMacHelper macAdhoc;
                macAdhoc.SetType("ns3::AdhocWifiMac");
                apDev[h] = wifi.Install(phy, macAdhoc, nodes.Get(h));
                staDev[h + 1] = wifi.Install(phy, macAdhoc, nodes.Get(h + 1));
            }
        }

        // One subnet per BSS: each BSS is its own L2 broadcast domain, so a flat subnet
        // would make IP believe distant nodes are on-link and ARP would never reach them.
        vector<Ipv4InterfaceContainer> ifc(hops);
        for (uint32_t h = 0; h < hops; ++h)
        {
            addr.SetBase(Ipv4Address(("10.1." + to_string(h + 1) + ".0").c_str()),
                         "255.255.255.0");
            NetDeviceContainer both(apDev[h], staDev[h + 1]);
            ifc[h] = addr.Assign(both); // .GetAddress(0) = AP (node h), (1) = STA (node h+1)
        }
        dstAddr = ifc[hops - 1].GetAddress(1);
        for (uint32_t i = 0; i + 1 < N; ++i)
        {
            nextHop[i] = ifc[i].GetAddress(1);
            apIface[i] = (i == 0) ? 1 : 2; // node 0 has one radio; relays are upper(1) then lower(2)
        }
    }

    for (uint32_t i = 0; i + 1 < N; ++i)
    {
        Ptr<Ipv4StaticRouting> r = srh.GetStaticRouting(nodes.Get(i)->GetObject<Ipv4>());
        r->AddHostRouteTo(dstAddr, nextHop[i], perHopChannel ? apIface[i] : 1);
    }

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, PORT));
    onoff.SetConstantRate(DataRate(rate), 700);
    ApplicationContainer a = onoff.Install(nodes.Get(0));
    a.Start(Seconds(3.0)); // late enough for every BSS to associate
    a.Stop(Seconds(11.0));
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), PORT));
    ApplicationContainer s = sink.Install(nodes.Get(N - 1));
    s.Start(Seconds(1.0));
    s.Stop(Seconds(12.0));

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();
    Simulator::Stop(Seconds(12.5));
    Simulator::Run();
    Result r = Collect(fm, fmh, PORT);
    Simulator::Destroy();
    return r;
}

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t hops = 4;
    double spacing = 50.0;
    string rate = "2Mbps";
    uint32_t nChannels = 0; // 0 = one channel per hop (no inter-hop interference)
    double baselineC = 4.53; // formula-gate's calibrated CSMA constant, ms/hop at 700 B
    CommandLine cmd;
    cmd.AddValue("hops", "Number of hops in the chain", hops);
    cmd.AddValue("spacing", "Distance between adjacent nodes (m)", spacing);
    cmd.AddValue("rate", "Offered rate of the single flow", rate);
    cmd.AddValue("nChannels", "Channels to cycle over in BSS mode (0 = one per hop)", nChannels);
    cmd.AddValue("baselineC", "formula-gate CSMA constant ms/hop, for reference", baselineC);
    cmd.Parse(argc, argv);

    if (nChannels == 0)
    {
        nChannels = hops;
    }

    cout << fixed << setprecision(3);
    cout << "=== hop-delay gate: " << hops << "-hop chain, " << spacing << " m, " << rate
         << " ===\n";
    cout << "BSS mode cycles over " << nChannels << " channel(s); each BSS has 1 STA (SU-OFDMA).\n";

    Result adhoc = RunChain(hops, spacing, rate, Mode::ADHOC, nChannels);
    Result adhocMc = RunChain(hops, spacing, rate, Mode::ADHOC_MC, nChannels);
    Result bss = RunChain(hops, spacing, rate, Mode::BSS, nChannels);

    cout << "\n  mode      delay(ms)   per-hop(ms)   thpt(Mb/s)   loss(%)   rxPkts\n";
    auto row = [&](const char* name, const Result& r) {
        cout << "  " << setw(8) << name << "   " << setw(8) << r.delayMs << "   " << setw(9)
             << (r.delayMs / hops) << "   " << setw(9) << r.thptMbps << "   " << setw(6)
             << r.lossPct << "   " << r.rxPackets << "\n";
    };
    row("adhoc", adhoc);       // 1 radio, 1 channel  = today's harness
    row("adhoc-mc", adhocMc);  // same radios/channels as bss, ad-hoc MAC = the control
    row("bss", bss);           // + AP scheduling

    cout << "\nreference: formula-gate CSMA constant C = " << baselineC << " ms/hop\n";
    if (adhoc.rxPackets == 0 || adhocMc.rxPackets == 0 || bss.rxPackets == 0)
    {
        cout << "\nINCONCLUSIVE: a mode delivered no packets (check association / routes)\n";
        return 1;
    }

    // Split the total gain into its two causes, so the thesis does not credit OFDMA for
    // what is really extra spectrum.
    cout << "\nattribution of the delay gain (lower is better):\n";
    cout << "  extra spectrum (adhoc -> adhoc-mc) : " << adhoc.delayMs << " -> " << adhocMc.delayMs
         << " ms  (" << (adhoc.delayMs / adhocMc.delayMs) << "x)\n";
    cout << "  AP scheduling  (adhoc-mc -> bss)   : " << adhocMc.delayMs << " -> " << bss.delayMs
         << " ms  (" << (adhocMc.delayMs / bss.delayMs) << "x)\n";
    cout << "  combined       (adhoc -> bss)      : " << adhoc.delayMs << " -> " << bss.delayMs
         << " ms  (" << (adhoc.delayMs / bss.delayMs) << "x)\n";

    // Only the scheduling half justifies the BSS rework; the spectrum half is available to a
    // plain multi-channel ad-hoc mesh, which is far cheaper to build.
    bool schedulingHelps = bss.delayMs < adhocMc.delayMs * 0.8;
    cout << "\n--- verdict ---\n";
    cout << "AP scheduling alone beats multi-channel ad-hoc by >20%: "
         << (schedulingHelps ? "PASS" : "FAIL") << "\n";
    cout << "per-hop: adhoc " << adhoc.delayMs / hops << " / adhoc-mc " << adhocMc.delayMs / hops
         << " / bss " << bss.delayMs / hops << " ms\n";
    return schedulingHelps ? 0 : 1;
}
