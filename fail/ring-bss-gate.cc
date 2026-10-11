// Gate T3 (DualChannel.md): does the grid-derived BSS structure survive CYCLES? The GA's tree is
// acyclic, but DualChannel.md §2 builds the BSS/channel structure over the physical topology, which
// is full of loops. So the structure must stand on its own on a cyclic graph, with no GA involved.
//
// Ring of N nodes, checkerboard roles: even index = AP, odd index = STA. Each AP serves its two
// ring neighbours, so every STA belongs to TWO BSSs at once and needs one radio per BSS -- the
// "checkerboard" row of §2's radios-vs-routes table, exercised for real.
//
//        n0(AP) --- n1(STA) --- n2(AP) --- n3(STA) --- n4(AP) --- n5(STA) --- back to n0
//        \____BSS0____/          \____BSS1____/         \____BSS2____/
//         (n1 and n5 are in BSS0; n1 also in BSS1; n5 also in BSS2)
//
// CHANNEL PARITY -- the point of the test. B = N/2 BSSs sit in a cycle of length B (two BSSs are
// adjacent when they share a STA, and that STA's two radios must be on different channels). So:
//   B even -> 2 channels suffice        B odd -> 2-colouring is impossible, 3 are needed
// N=6 gives B=3 (odd, needs 3); N=8 gives B=4 (even, needs 2). --channels forces a count so the
// odd case can be shown failing: some STA then gets both radios on one channel, which violates the
// colouring paper's collision rule 3 and self-interferes.
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/wifi-module.h"

#include <csignal>
#include <cmath>
#include <execinfo.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <unistd.h>
#include <vector>

using namespace ns3;
using namespace std;

namespace {

const vector<uint32_t> CH80 = {42, 58, 106, 122, 138, 155, 171};
const uint16_t PORT = 8001;

// No gdb in this environment: dump a native backtrace (and the sim clock) on SIGSEGV.
void
SegvHandler(int)
{
    void* bt[64];
    int n = backtrace(bt, 64);
    fprintf(stderr, "\n*** SIGSEGV at sim t = %.6f s ***\n", Simulator::Now().GetSeconds());
    backtrace_symbols_fd(bt, n, STDERR_FILENO);
    _exit(139);
}

Ptr<MultiModelSpectrumChannel>
MakeChannel()
{
    Ptr<MultiModelSpectrumChannel> c = CreateObject<MultiModelSpectrumChannel>();
    c->AddPropagationLossModel(CreateObject<LogDistancePropagationLossModel>());
    c->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    return c;
}

} // namespace

int
main(int argc, char* argv[])
{
    signal(SIGSEGV, SegvHandler);

    uint32_t N = 6;         // ring size (even: roles alternate AP/STA)
    uint32_t channels = 0;  // 0 = auto (2 if B even else 3); force a value to break it on purpose
    double radius = 45.0;   // ring radius; neighbour spacing = 2*radius*sin(pi/N)
    string rate = "10Mbps";
    bool hop1 = false; // send only node0->node1 inside BSS0 (isolates cross-BSS forwarding)
    bool mu = true;    // MU scheduler on the ring APs
    bool toAp = false; // 2-hop target is BSS1's AP instead of another STA
    int mcs = -1;      // -1 = IdealWifiManager, else fixed HeMcs<n>
    string phyModel = "spectrum"; // "spectrum" (OFDMA-capable) or "yans" (range control)
    double txPower = 16.0206;     // dBm; ns-3 WifiPhy default
    CommandLine cmd;
    cmd.AddValue("nodes", "Ring size (must be even)", N);
    cmd.AddValue("channels", "Channels to cycle over (0 = auto by parity)", channels);
    cmd.AddValue("radius", "Ring radius (m)", radius);
    cmd.AddValue("rate", "Offered rate of the test flow", rate);
    cmd.AddValue("hop1", "Single-hop test inside BSS0 only", hop1);
    cmd.AddValue("mu", "Install MU scheduler on the ring APs", mu);
    cmd.AddValue("toAp", "2-hop destination is the next BSS's AP (no STA->STA relay)", toAp);
    cmd.AddValue("mcs", "Fixed HeMcs index (-1 = IdealWifiManager)", mcs);
    cmd.AddValue("phy", "PHY model: spectrum | yans", phyModel);
    cmd.AddValue("txPower", "Tx power (dBm)", txPower);
    cmd.Parse(argc, argv);

    if (N < 4 || N % 2 != 0)
    {
        cout << "ring size must be even and >= 4\n";
        return 1;
    }
    const uint32_t B = N / 2; // number of BSSs
    const uint32_t needed = (B % 2 == 0) ? 2 : 3;
    if (channels == 0)
    {
        channels = needed;
    }

    const double spacing = 2.0 * radius * sin(M_PI / N);
    cout << fixed << setprecision(3);
    cout << "=== ring BSS gate: " << N << " nodes, " << B << " BSSs, neighbour spacing "
         << spacing << " m ===\n";
    cout << "BSS adjacency cycle length " << B << " (" << (B % 2 ? "ODD" : "even")
         << ") -> channels needed = " << needed << ", using " << channels << "\n";

    NodeContainer nodes;
    nodes.Create(N);

    MobilityHelper mob;
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < N; ++i)
    {
        double a = 2.0 * M_PI * i / N;
        pos->Add(Vector(radius * cos(a), radius * sin(a), 0.0));
    }
    mob.SetPositionAllocator(pos);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    if (mcs < 0)
    {
        wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    }
    else
    {
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                     "DataMode",
                                     StringValue("HeMcs" + to_string(mcs)),
                                     "ControlMode",
                                     StringValue("OfdmRate24Mbps"));
    }

    InternetStackHelper internet;
    Ipv4StaticRoutingHelper srh;
    internet.SetRoutingHelper(srh);
    internet.Install(nodes);
    Ipv4AddressHelper addr;

    const bool useSpectrum = (phyModel != "yans");
    vector<Ptr<MultiModelSpectrumChannel>> chans(channels);
    vector<Ptr<YansWifiChannel>> ychans(channels);
    for (uint32_t c = 0; c < channels; ++c)
    {
        if (useSpectrum)
        {
            chans[c] = MakeChannel();
        }
        else
        {
            YansWifiChannelHelper h;
            h.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
            h.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
            ychans[c] = h.Create();
        }
    }

    // addrOf[{node, bss}] and per-node radio channel list (to detect rule-3 violations).
    map<pair<uint32_t, uint32_t>, Ipv4Address> addrOf;
    map<pair<uint32_t, uint32_t>, uint32_t> ifaceOf;
    vector<vector<uint32_t>> nodeChans(N);
    vector<Ptr<StaWifiMac>> staMacs;
    vector<pair<uint32_t, uint32_t>> staOwner; // (node, bss) for each entry in staMacs

    for (uint32_t k = 0; k < B; ++k)
    {
        uint32_t ap = 2 * k;
        uint32_t left = (2 * k + N - 1) % N;
        uint32_t right = (2 * k + 1) % N;
        uint32_t c = k % channels;

        unique_ptr<WifiPhyHelper> phyPtr;
        if (useSpectrum)
        {
            auto p = make_unique<SpectrumWifiPhyHelper>();
            p->SetChannel(chans[c]);
            phyPtr = move(p);
        }
        else
        {
            auto p = make_unique<YansWifiPhyHelper>();
            p->SetChannel(ychans[c]);
            phyPtr = move(p);
        }
        WifiPhyHelper& phy = *phyPtr;
        phy.Set("ChannelSettings",
                StringValue("{" + to_string(CH80[c % CH80.size()]) + ", 80, BAND_5GHZ, 0}"));
        phy.Set("TxPowerStart", DoubleValue(txPower));
        phy.Set("TxPowerEnd", DoubleValue(txPower));
        Ssid ssid = Ssid("ring-bss-" + to_string(k));

        WifiMacHelper macAp;
        if (mu)
        {
            macAp.SetMultiUserScheduler("ns3::RrMultiUserScheduler");
        }
        macAp.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        NetDeviceContainer dAp = wifi.Install(phy, macAp, nodes.Get(ap));

        WifiMacHelper macSta;
        macSta.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        NetDeviceContainer dSta =
            wifi.Install(phy, macSta, NodeContainer(nodes.Get(left), nodes.Get(right)));

        nodeChans[ap].push_back(c);
        nodeChans[left].push_back(c);
        nodeChans[right].push_back(c);

        addr.SetBase(Ipv4Address(("10.3." + to_string(k + 1) + ".0").c_str()), "255.255.255.0");
        NetDeviceContainer both(dAp, dSta);
        Ipv4InterfaceContainer ifc = addr.Assign(both); // 0=ap, 1=left, 2=right

        addrOf[{ap, k}] = ifc.GetAddress(0);
        addrOf[{left, k}] = ifc.GetAddress(1);
        addrOf[{right, k}] = ifc.GetAddress(2);
        // Ask Ipv4 for the interface index: guessing it from GetNDevices() is off by one, because
        // the stack installs loopback as BOTH device 0 and interface 0.
        ifaceOf[{ap, k}] = nodes.Get(ap)->GetObject<Ipv4>()->GetInterfaceForDevice(dAp.Get(0));
        ifaceOf[{left, k}] = nodes.Get(left)->GetObject<Ipv4>()->GetInterfaceForDevice(dSta.Get(0));
        ifaceOf[{right, k}] =
            nodes.Get(right)->GetObject<Ipv4>()->GetInterfaceForDevice(dSta.Get(1));

        for (uint32_t i = 0; i < dSta.GetN(); ++i)
        {
            staMacs.push_back(DynamicCast<StaWifiMac>(DynamicCast<WifiNetDevice>(dSta.Get(i))->GetMac()));
            staOwner.push_back({i == 0 ? left : right, k});
        }
    }

    // Rule 3 check: no node may hold two radios on the same channel.
    uint32_t violations = 0;
    for (uint32_t i = 0; i < N; ++i)
    {
        map<uint32_t, uint32_t> seen;
        for (uint32_t c : nodeChans[i])
        {
            if (++seen[c] > 1)
            {
                cout << "  RULE-3 VIOLATION: node" << i << " has 2 radios on channel index " << c
                     << "\n";
                ++violations;
            }
        }
    }

    // Traffic across a BSS boundary: node0 (AP of BSS0) -> node3 (a STA of BSS1), relayed by
    // node1, which belongs to BOTH BSS0 and BSS1. This is the whole point -- a node bridging two
    // BSSs is what makes the cyclic structure carry traffic.
    // node3 = a STA of BSS1 -> the second hop is STA->STA, which the AP must relay.
    // node2 = the AP of BSS1  -> the second hop is STA->AP, no relaying.
    uint32_t dstNode = hop1 ? 1 : (toAp ? 2 : 3);
    Ipv4Address dst = hop1 ? addrOf[{1, 0}] : (toAp ? addrOf[{2, 1}] : addrOf[{3, 1}]);
    if (!hop1)
    {
        Ptr<Ipv4StaticRouting> r0 = srh.GetStaticRouting(nodes.Get(0)->GetObject<Ipv4>());
        r0->AddHostRouteTo(dst, addrOf[{1, 0}], ifaceOf[{0, 0}]);
    }

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dst, PORT));
    onoff.SetConstantRate(DataRate(rate), 700);
    ApplicationContainer a = onoff.Install(nodes.Get(0));
    a.Start(Seconds(4.0)); // ring associations take longer than a single BSS
    a.Stop(Seconds(11.0));
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), PORT));
    ApplicationContainer s = sink.Install(nodes.Get(dstNode));
    s.Start(Seconds(1.0));
    s.Stop(Seconds(12.0));

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();
    Simulator::Stop(Seconds(12.5));
    Simulator::Run();

    cout << "\nassociations (each STA belongs to 2 BSSs):\n";
    uint32_t assoc = 0;
    for (size_t i = 0; i < staMacs.size(); ++i)
    {
        bool ok = staMacs[i]->IsAssociated();
        assoc += ok ? 1 : 0;
        cout << "  node" << staOwner[i].first << " in BSS" << staOwner[i].second << ": "
             << (ok ? "ASSOCIATED" : "NOT associated") << "\n";
    }

    uint64_t tx = 0, rx = 0, rxBytes = 0;
    double delaySum = 0.0;
    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    for (const auto& [id, st] : fm->GetFlowStats())
    {
        if (cl->FindFlow(id).destinationPort != PORT)
        {
            continue;
        }
        tx += st.txPackets;
        rx += st.rxPackets;
        rxBytes += st.rxBytes;
        delaySum += st.delaySum.GetSeconds();
    }
    double delayMs = (rx > 0) ? delaySum / rx * 1e3 : 0.0;

    cout << "\nnode0 -> node3 (2 hops, bridged by node1 which is in BSS0 and BSS1):\n";
    cout << "  tx=" << tx << " rx=" << rx << " bytes=" << rxBytes << " delay=" << delayMs
         << " ms  loss=" << (tx ? 100.0 * (tx - rx) / tx : 0.0) << " %\n";

    cout << "\n--- verdict ---\n";
    cout << "channel parity respected (no node with 2 radios on one channel): "
         << (violations == 0 ? "PASS" : "FAIL") << "  (" << violations << " violations)\n";
    cout << "every STA associated in BOTH its BSSs: "
         << (assoc == staMacs.size() ? "PASS" : "FAIL") << "  (" << assoc << "/" << staMacs.size()
         << ")\n";
    cout << "traffic crosses a BSS boundary around the ring: " << (rx > 0 ? "PASS" : "FAIL")
         << "\n";

    Simulator::Destroy();
    return (violations == 0 && assoc == staMacs.size() && rx > 0) ? 0 : 1;
}
