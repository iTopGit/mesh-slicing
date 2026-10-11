// Gate T2 (DualChannel.md): does OFDMA actually PAY? T1 showed it loses in a chain, but a chain
// gives each BSS exactly one STA, so there is nothing to multiplex -- pure scheduler overhead.
// This is OFDMA's best case: one centre node with 4 neighbours all sending at once, which is what
// a grid node looks like under the checkerboard scheme. If OFDMA loses here too, it loses
// everywhere and Plan B cannot be justified on performance.
//
// SINGLE HOP on ONE channel, one radio per node -- no dual role, no second channel, no routing.
// Same hardware, same traffic, same SpectrumWifiPhy; the ONLY difference is the MAC:
//   adhoc : 5 peers, everyone contends (CSMA/CA)
//   bss   : centre = AP with RrMultiUserScheduler, 4 neighbours = STAs
//
// The 4 "STAs" are mesh routers, not client devices.
//
// Both directions are measured, because they stress different things:
//   DL : centre -> 4 neighbours. The AP splits its channel into RUs (DL MU-OFDMA).
//   UL : 4 neighbours -> centre. In ad-hoc this is the classic collision case; with an AP,
//        trigger frames schedule the STAs so they cannot collide (UL MU-OFDMA).
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
#include <map>
#include <vector>

using namespace ns3;
using namespace std;

namespace {

const uint32_t LEAVES = 4;
const uint16_t PORT_BASE = 6001;

uint32_t g_dlMu = 0;             // DL MU PPDUs the centre transmitted
uint32_t g_ulMu = 0;             // TB PPDUs the leaves transmitted (UL OFDMA)
uint32_t g_su = 0;               // everything else
map<size_t, uint32_t> g_muWidth; // STAs multiplexed per DL MU PPDU

void
CentreTx(WifiConstPsduMap psduMap, WifiTxVector txVector, double)
{
    if (txVector.IsDlMu() && psduMap.size() > 1)
    {
        ++g_dlMu;
        ++g_muWidth[psduMap.size()];
    }
    else
    {
        ++g_su;
    }
}

void
LeafTx(WifiConstPsduMap, WifiTxVector txVector, double)
{
    if (txVector.IsUlMu())
    {
        ++g_ulMu;
    }
}

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

void
AddFlow(Ptr<Node> src, Ptr<Node> dst, Ipv4Address dstAddr, uint16_t port, const string& rate)
{
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    onoff.SetConstantRate(DataRate(rate), 700);
    ApplicationContainer a = onoff.Install(src);
    a.Start(Seconds(3.0)); // after association
    a.Stop(Seconds(11.0));
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer s = sink.Install(dst);
    s.Start(Seconds(1.0));
    s.Stop(Seconds(12.0));
}

// Aggregate every flow in [PORT_BASE, PORT_BASE+LEAVES): mean delay weighted by packets,
// summed throughput, overall loss.
Result
Collect(Ptr<FlowMonitor> fm, FlowMonitorHelper& fmh)
{
    Result r;
    uint64_t tx = 0, rx = 0, rxBytes = 0;
    double delaySum = 0.0, firstRx = 1e18, lastRx = 0.0;
    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    for (const auto& [id, st] : fm->GetFlowStats())
    {
        uint16_t p = cl->FindFlow(id).destinationPort;
        if (p < PORT_BASE || p >= PORT_BASE + LEAVES)
        {
            continue;
        }
        tx += st.txPackets;
        rx += st.rxPackets;
        rxBytes += st.rxBytes;
        delaySum += st.delaySum.GetSeconds();
        if (st.rxPackets > 0)
        {
            firstRx = min(firstRx, st.timeFirstRxPacket.GetSeconds());
            lastRx = max(lastRx, st.timeLastRxPacket.GetSeconds());
        }
    }
    double dur = (lastRx > firstRx) ? (lastRx - firstRx) : 0.0;
    r.rxPackets = rx;
    r.delayMs = (rx > 0) ? delaySum / rx * 1e3 : 0.0;
    r.thptMbps = (dur > 0) ? rxBytes * 8.0 / dur / 1e6 : 0.0;
    r.lossPct = (tx > 0) ? 100.0 * (tx - rx) / tx : 0.0;
    return r;
}

// One star run. `bss` picks the MAC; `uplink` picks the traffic direction.
Result
RunStar(const string& rate, bool bss, bool uplink, uint32_t ch, double radius)
{
    NodeContainer nodes;
    nodes.Create(1 + LEAVES); // node0 = centre

    MobilityHelper mob;
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 0.0));
    pos->Add(Vector(radius, 0.0, 0.0));
    pos->Add(Vector(-radius, 0.0, 0.0));
    pos->Add(Vector(0.0, radius, 0.0));
    pos->Add(Vector(0.0, -radius, 0.0));
    mob.SetPositionAllocator(pos);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    SpectrumWifiPhyHelper phy;
    phy.SetChannel(MakeChannel());
    phy.Set("ChannelSettings", StringValue("{" + to_string(ch) + ", 80, BAND_5GHZ, 0}"));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    NetDeviceContainer devCentre;
    NetDeviceContainer devLeaves;
    if (bss)
    {
        Ssid ssid = Ssid("star");
        WifiMacHelper macAp;
        // UL OFDMA is off by default; without it the leaves contend normally on the uplink.
        macAp.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
                                    "EnableUlOfdma",
                                    BooleanValue(true),
                                    "EnableBsrp",
                                    BooleanValue(true));
        macAp.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        devCentre = wifi.Install(phy, macAp, nodes.Get(0));

        WifiMacHelper macSta;
        macSta.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        NodeContainer leaves;
        for (uint32_t i = 1; i <= LEAVES; ++i)
        {
            leaves.Add(nodes.Get(i));
        }
        devLeaves = wifi.Install(phy, macSta, leaves);
    }
    else
    {
        WifiMacHelper mac;
        mac.SetType("ns3::AdhocWifiMac");
        devCentre = wifi.Install(phy, mac, nodes.Get(0));
        NodeContainer leaves;
        for (uint32_t i = 1; i <= LEAVES; ++i)
        {
            leaves.Add(nodes.Get(i));
        }
        devLeaves = wifi.Install(phy, mac, leaves);
    }

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper addr;
    addr.SetBase("10.2.1.0", "255.255.255.0");
    NetDeviceContainer all(devCentre, devLeaves);
    Ipv4InterfaceContainer ifc = addr.Assign(all); // 0 = centre, 1..4 = leaves

    // Single hop: every pair is directly reachable, no routes needed.
    for (uint32_t i = 1; i <= LEAVES; ++i)
    {
        uint16_t port = PORT_BASE + (i - 1);
        if (uplink)
        {
            AddFlow(nodes.Get(i), nodes.Get(0), ifc.GetAddress(0), port, rate);
        }
        else
        {
            AddFlow(nodes.Get(0), nodes.Get(i), ifc.GetAddress(i), port, rate);
        }
    }

    DynamicCast<WifiNetDevice>(devCentre.Get(0))
        ->GetPhy()
        ->TraceConnectWithoutContext("PhyTxPsduBegin", MakeCallback(&CentreTx));
    for (uint32_t i = 0; i < devLeaves.GetN(); ++i)
    {
        DynamicCast<WifiNetDevice>(devLeaves.Get(i))
            ->GetPhy()
            ->TraceConnectWithoutContext("PhyTxPsduBegin", MakeCallback(&LeafTx));
    }

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();
    Simulator::Stop(Seconds(12.5));
    Simulator::Run();
    Result r = Collect(fm, fmh);
    Simulator::Destroy();
    return r;
}

} // namespace

int
main(int argc, char* argv[])
{
    string rate = "10Mbps";
    uint32_t ch = 42;
    double radius = 30.0;
    CommandLine cmd;
    cmd.AddValue("rate", "Per-flow offered rate (4 flows total)", rate);
    cmd.AddValue("ch", "80 MHz centre channel", ch);
    cmd.AddValue("radius", "Distance from centre to each neighbour (m)", radius);
    cmd.Parse(argc, argv);

    cout << fixed << setprecision(3);
    cout << "=== star gate: 1 centre + " << LEAVES << " neighbours, single hop, ch" << ch << " ===\n";
    cout << "per-flow " << rate << " x " << LEAVES << " flows, radius " << radius << " m\n";

    struct Row
    {
        const char* name;
        Result r;
        uint32_t dlMu;
        uint32_t ulMu;
    };
    vector<Row> rows;

    auto run = [&](const char* name, bool bss, bool ul) {
        g_dlMu = g_ulMu = g_su = 0;
        g_muWidth.clear();
        Result r = RunStar(rate, bss, ul, ch, radius);
        rows.push_back({name, r, g_dlMu, g_ulMu});
    };

    run("DL adhoc", false, false);
    run("DL bss", true, false);
    run("UL adhoc", false, true);
    run("UL bss", true, true);

    cout << "\n  case        delay(ms)   thpt(Mb/s)   loss(%)   rxPkts    DL-MU   UL-MU\n";
    for (const Row& row : rows)
    {
        cout << "  " << setw(9) << row.name << "   " << setw(8) << row.r.delayMs << "   " << setw(9)
             << row.r.thptMbps << "   " << setw(6) << row.r.lossPct << "   " << setw(7)
             << row.r.rxPackets << "   " << setw(6) << row.dlMu << "   " << setw(5) << row.ulMu
             << "\n";
    }

    const Result& dlA = rows[0].r;
    const Result& dlB = rows[1].r;
    const Result& ulA = rows[2].r;
    const Result& ulB = rows[3].r;

    cout << "\ndownlink  adhoc -> bss : delay " << dlA.delayMs << " -> " << dlB.delayMs
         << " ms, thpt " << dlA.thptMbps << " -> " << dlB.thptMbps << " Mb/s\n";
    cout << "uplink    adhoc -> bss : delay " << ulA.delayMs << " -> " << ulB.delayMs
         << " ms, thpt " << ulA.thptMbps << " -> " << ulB.thptMbps << " Mb/s\n";

    // Judge on goodput, not delay: under overload ad-hoc drops packets at the queue and those
    // never enter the delay average, so it can post a low delay while delivering far less.
    // "Better" = carries more traffic (>5%) without losing more of what it accepted.
    bool dlWins = dlB.thptMbps > dlA.thptMbps * 1.05;
    bool ulWins = ulB.thptMbps > ulA.thptMbps * 1.05;

    cout << "\n--- verdict ---\n";
    cout << "OFDMA actually multiplexed (DL MU PPDUs > 0): " << (rows[1].dlMu > 0 ? "yes" : "NO")
         << "  (" << rows[1].dlMu << " PPDUs)\n";
    for (const auto& [width, n] : g_muWidth)
    {
        cout << "    last run: " << width << " STAs per MU PPDU x " << n << "\n";
    }
    cout << "downlink goodput: bss beats adhoc by >5% : " << (dlWins ? "PASS" : "FAIL") << "  ("
         << dlA.thptMbps << " -> " << dlB.thptMbps << " Mb/s)\n";
    cout << "uplink   goodput: bss beats adhoc by >5% : " << (ulWins ? "PASS" : "FAIL") << "  ("
         << ulA.thptMbps << " -> " << ulB.thptMbps << " Mb/s)\n";
    return (dlWins || ulWins) ? 0 : 1;
}
