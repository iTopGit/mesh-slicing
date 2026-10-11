// Gate: can one node be an AP and a STA at the SAME TIME (two radios, two channels), and does
// that give us real 802.11ax MU-OFDMA inside a mesh? Design in DualChannel.md. The harness's
// ad-hoc stack can never run a MultiUserScheduler ("can only be installed on HE APs"), so the
// whole dual-channel design rests on the dual-role node working. Topology:
//
//        node0 [AP ch36]
//           ^ STA ch36
//        node1 [STA ch36 | AP ch149]   <- the dual-role node
//           v ch149        v ch149
//      node2 [STA]      node3 [STA]
//
// Traffic runs node0 -> node2 and node0 -> node3, so node1 receives as a STA and forwards as an
// AP with two STAs queued at once -- the only condition under which RrMultiUserScheduler builds
// an MU PPDU. Proof of OFDMA is the PhyTxPsduBegin trace on node1's AP radio: a WifiConstPsduMap
// with >1 entry and txVector.IsDlMu() is a real RU split, and the map keys are the STAs' AIDs.
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/wifi-module.h"

#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <set>
#include <vector>

using namespace ns3;
using namespace std;

namespace {

uint32_t g_su = 0;                  // SU PPDUs sent by node1's AP
uint32_t g_mu = 0;                  // DL MU (OFDMA) PPDUs
set<uint16_t> g_aids;               // STA-IDs (AIDs) seen inside MU PPDUs
map<size_t, uint32_t> g_muWidth;    // how many STAs shared each MU PPDU

uint32_t g_ap0Tx = 0;   // PPDUs node0's AP puts on air (is it beaconing at all?)
uint32_t g_sta1Rx = 0;  // PPDUs node1's STA radio successfully receives

double g_ap0LastTx = -1.0;
double g_sta1LastRx = -1.0;

void
Ap0Tx(WifiConstPsduMap, WifiTxVector, double)
{
    ++g_ap0Tx;
    g_ap0LastTx = Simulator::Now().GetSeconds();
}

void
Sta1Rx(Ptr<const Packet>, RxPowerWattPerChannelBand)
{
    ++g_sta1Rx;
    g_sta1LastRx = Simulator::Now().GetSeconds();
}

map<string, double> g_state; // node1 STA PHY: seconds spent in each PHY state

vector<string> g_stateLog; // last state transitions, to see what the PHY froze in

void
Sta1State(Time start, Time dur, WifiPhyState st)
{
    ostringstream o;
    o << st;
    g_state[o.str()] += dur.GetSeconds();
    ostringstream line;
    line << "    [" << start.GetSeconds() << "s +" << dur.GetSeconds() << "s] " << o.str();
    g_stateLog.push_back(line.str());
}

void
Sta1Assoc(Mac48Address)
{
    cout << "  [" << Simulator::Now().GetSeconds() << "s] node1 STA ASSOCIATED\n";
}

void
Sta1DeAssoc(Mac48Address)
{
    cout << "  [" << Simulator::Now().GetSeconds() << "s] node1 STA LOST association\n";
}

void
PsduTx(WifiConstPsduMap psduMap, WifiTxVector txVector, double)
{
    if (txVector.IsDlMu() && psduMap.size() > 1)
    {
        ++g_mu;
        ++g_muWidth[psduMap.size()];
        for (const auto& [staId, psdu] : psduMap)
        {
            g_aids.insert(staId);
        }
    }
    else
    {
        ++g_su;
    }
}

// One UDP flow src->dst, plus its sink.
void
AddFlow(Ptr<Node> src, Ptr<Node> dst, Ipv4Address dstAddr, uint16_t port, const string& rate)
{
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    onoff.SetConstantRate(DataRate(rate), 700);
    ApplicationContainer a = onoff.Install(src);
    a.Start(Seconds(2.0));
    a.Stop(Seconds(7.0));

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer s = sink.Install(dst);
    s.Start(Seconds(1.0));
    s.Stop(Seconds(8.0));
}

} // namespace

int
main(int argc, char* argv[])
{
    string rate = "20Mbps";
    uint32_t chUp = 42;    // node0 <-> node1 (80 MHz centre channels: 42/58/106/122/138/155)
    uint32_t chDown = 155; // node1 <-> node2,node3
    bool dualRole = true;  // false: skip node1's AP radio, to test the STA half alone
    bool flows = true;     // false: association only, no traffic (isolates radio vs load)
    uint32_t maxMissed = 10;   // ns-3 default; kept as a knob, no workaround needed
    bool muUpper = true;       // MU scheduler on node0's AP (the upper BSS has only 1 STA)
    CommandLine cmd;
    cmd.AddValue("rate", "Per-flow offered rate", rate);
    cmd.AddValue("chUp", "Channel number for the upper BSS", chUp);
    cmd.AddValue("chDown", "Channel number for the lower BSS", chDown);
    cmd.AddValue("dualRole", "Install node1's AP radio (false = STA only, isolates the conflict)",
                 dualRole);
    cmd.AddValue("flows", "Run the UDP flows (false = association only)", flows);
    cmd.AddValue("maxMissed", "StaWifiMac::MaxMissedBeacons (ns-3 default is 10)", maxMissed);
    cmd.AddValue("muUpper", "Install a MU scheduler on node0's AP too", muUpper);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::StaWifiMac::MaxMissedBeacons", UintegerValue(maxMissed));

    NodeContainer nodes;
    nodes.Create(4);

    MobilityHelper mob;
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0.0, 0.0, 0.0));   // node0
    pos->Add(Vector(30.0, 0.0, 0.0));  // node1 (in range of all)
    pos->Add(Vector(60.0, 0.0, 0.0));  // node2
    pos->Add(Vector(30.0, 30.0, 0.0)); // node3
    mob.SetPositionAllocator(pos);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    // MUST be SpectrumWifiPhy, not Yans: OFDMA splits a channel into per-subcarrier RUs and the
    // Yans model treats a channel as one indivisible unit, so every MU PPDU fails silently
    // (examples/wireless/wifi-he-network.cc: "SpectrumWifiPhy is required for ... OFDMA").
    // Separate channel objects per band: orthogonal channels must not hear each other at all.
    auto makeChannel = []() {
        Ptr<MultiModelSpectrumChannel> c = CreateObject<MultiModelSpectrumChannel>();
        c->AddPropagationLossModel(CreateObject<LogDistancePropagationLossModel>());
        c->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
        return c;
    };
    Ptr<MultiModelSpectrumChannel> chanUp = makeChannel();
    Ptr<MultiModelSpectrumChannel> chanDown = makeChannel();

    SpectrumWifiPhyHelper phyUp;
    phyUp.SetChannel(chanUp);
    phyUp.Set("ChannelSettings", StringValue("{" + to_string(chUp) + ", 80, BAND_5GHZ, 0}"));
    SpectrumWifiPhyHelper phyDown;
    phyDown.SetChannel(chanDown);
    phyDown.Set("ChannelSettings", StringValue("{" + to_string(chDown) + ", 80, BAND_5GHZ, 0}"));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    Ssid ssidUp = Ssid("bss-up");
    Ssid ssidDown = Ssid("bss-down");

    // --- upper BSS: node0 = AP, node1 = STA ---
    WifiMacHelper macApUp;
    if (muUpper)
    {
        macApUp.SetMultiUserScheduler("ns3::RrMultiUserScheduler");
    }
    macApUp.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidUp));
    NetDeviceContainer devAp0 = wifi.Install(phyUp, macApUp, nodes.Get(0));

    WifiMacHelper macSta;
    macSta.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidUp));
    NetDeviceContainer devSta1 = wifi.Install(phyUp, macSta, nodes.Get(1));

    // --- lower BSS: node1 = AP (its SECOND radio), node2 + node3 = STAs ---
    NetDeviceContainer devAp1;
    NetDeviceContainer devSta23;
    if (dualRole)
    {
        WifiMacHelper macApDown;
        macApDown.SetMultiUserScheduler("ns3::RrMultiUserScheduler");
        macApDown.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidDown));
        devAp1 = wifi.Install(phyDown, macApDown, nodes.Get(1));

        WifiMacHelper macStaDown;
        macStaDown.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidDown));
        devSta23 = wifi.Install(phyDown, macStaDown, NodeContainer(nodes.Get(2), nodes.Get(3)));
    }

    InternetStackHelper internet;
    Ipv4StaticRoutingHelper srh;
    internet.SetRoutingHelper(srh);
    internet.Install(nodes);

    // One subnet per BSS (each BSS is its own L2 broadcast domain).
    Ipv4AddressHelper addr;
    addr.SetBase("10.1.1.0", "255.255.255.0");
    NetDeviceContainer bssUp(devAp0, devSta1);
    Ipv4InterfaceContainer ifUp = addr.Assign(bssUp);
    Ipv4InterfaceContainer ifDown;
    if (dualRole)
    {
        addr.SetBase("10.1.2.0", "255.255.255.0");
        NetDeviceContainer bssDown(devAp1, devSta23);
        ifDown = addr.Assign(bssDown);
    }

    if (dualRole)
    {
        // node0: everything in the lower BSS goes via node1's STA-side address.
        Ptr<Ipv4StaticRouting> r0 = srh.GetStaticRouting(nodes.Get(0)->GetObject<Ipv4>());
        r0->AddNetworkRouteTo(Ipv4Address("10.1.2.0"), Ipv4Mask("255.255.255.0"),
                              ifUp.GetAddress(1), 1);
        // node2/node3: default via node1's AP-side address.
        for (uint32_t i = 1; i <= 2; ++i)
        {
            Ptr<Ipv4StaticRouting> r = srh.GetStaticRouting(nodes.Get(i + 1)->GetObject<Ipv4>());
            r->SetDefaultRoute(ifDown.GetAddress(0), 1);
        }
        if (flows)
        {
            AddFlow(nodes.Get(0), nodes.Get(2), ifDown.GetAddress(1), 5001, rate);
            AddFlow(nodes.Get(0), nodes.Get(3), ifDown.GetAddress(2), 5002, rate);
        }
    }

    // Watch what node1's AP radio actually puts on the air.
    Ptr<WifiNetDevice> ap1;
    if (dualRole)
    {
        ap1 = DynamicCast<WifiNetDevice>(devAp1.Get(0));
        ap1->GetPhy()->TraceConnectWithoutContext("PhyTxPsduBegin", MakeCallback(&PsduTx));
    }
    DynamicCast<WifiNetDevice>(devAp0.Get(0))
        ->GetPhy()
        ->TraceConnectWithoutContext("PhyTxPsduBegin", MakeCallback(&Ap0Tx));
    DynamicCast<WifiNetDevice>(devSta1.Get(0))
        ->GetPhy()
        ->TraceConnectWithoutContext("PhyRxBegin", MakeCallback(&Sta1Rx));
    DynamicCast<WifiNetDevice>(devSta1.Get(0))
        ->GetPhy()
        ->GetState()
        ->TraceConnectWithoutContext("State", MakeCallback(&Sta1State));
    Ptr<WifiMac> sta1MacEarly = DynamicCast<WifiNetDevice>(devSta1.Get(0))->GetMac();
    sta1MacEarly->TraceConnectWithoutContext("Assoc", MakeCallback(&Sta1Assoc));
    sta1MacEarly->TraceConnectWithoutContext("DeAssoc", MakeCallback(&Sta1DeAssoc));
    cout << "--- node1 STA association timeline ---\n";

    cout << "--- device map ---\n";
    for (uint32_t n = 0; n < nodes.GetN(); ++n)
    {
        Ptr<Node> nd = nodes.Get(n);
        for (uint32_t d = 0; d < nd->GetNDevices(); ++d)
        {
            Ptr<WifiNetDevice> wd = DynamicCast<WifiNetDevice>(nd->GetDevice(d));
            if (!wd)
            {
                continue;
            }
            const auto& oc = wd->GetPhy()->GetOperatingChannel();
            cout << "  node" << n << " dev" << d << "  " << wd->GetMac()->GetInstanceTypeId().GetName()
                 << "  ch=" << +oc.GetNumber() << " width=" << oc.GetWidth()
                 << " ssid=" << wd->GetMac()->GetSsid() << "\n";
        }
    }

    Simulator::Stop(Seconds(9.0));
    Simulator::Run();

    // --- assertion 1: the dual-role node associated upward AND serves downward ---
    Ptr<StaWifiMac> sta1 = DynamicCast<StaWifiMac>(DynamicCast<WifiNetDevice>(devSta1.Get(0))->GetMac());
    Ptr<StaWifiMac> sta2;
    Ptr<StaWifiMac> sta3;
    if (dualRole)
    {
        sta2 = DynamicCast<StaWifiMac>(DynamicCast<WifiNetDevice>(devSta23.Get(0))->GetMac());
        sta3 = DynamicCast<StaWifiMac>(DynamicCast<WifiNetDevice>(devSta23.Get(1))->GetMac());
    }
    bool up = sta1->IsAssociated();
    bool down = dualRole && sta2->IsAssociated() && sta3->IsAssociated();
    bool bothRoles = up && down;

    // --- did the AP actually get a scheduler? separates "misconfigured" from "no MU traffic" ---
    bool hasScheduler = dualRole && (ap1->GetMac()->GetObject<MultiUserScheduler>() != nullptr);

    // --- assertion 3: traffic really crossed the dual-role node ---
    uint64_t rx2 = 0;
    uint64_t rx3 = 0;
    if (dualRole && flows)
    {
        rx2 = DynamicCast<PacketSink>(nodes.Get(2)->GetApplication(0))->GetTotalRx();
        rx3 = DynamicCast<PacketSink>(nodes.Get(3)->GetApplication(0))->GetTotalRx();
    }
    bool forwarded = (rx2 > 0 && rx3 > 0);

    cout << fixed << setprecision(2);
    cout << "=== dual-role gate (node1 = STA on ch" << chUp << " + AP on ch" << chDown << ") ===\n";
    cout << "\nassociation:\n";
    cout << "  node1 STA -> node0 AP (upper BSS): " << (up ? "ASSOCIATED" : "NOT associated")
         << "  (AID " << (up ? sta1->GetAssociationId() : 0) << ")\n";
    cout << "  node2 STA -> node1 AP (lower BSS): "
         << (!dualRole ? "n/a" : (sta2->IsAssociated() ? "ASSOCIATED" : "NOT associated"))
         << "  (AID " << ((dualRole && sta2->IsAssociated()) ? sta2->GetAssociationId() : 0) << ")\n";
    cout << "  node3 STA -> node1 AP (lower BSS): "
         << (!dualRole ? "n/a" : (sta3->IsAssociated() ? "ASSOCIATED" : "NOT associated"))
         << "  (AID " << ((dualRole && sta3->IsAssociated()) ? sta3->GetAssociationId() : 0) << ")\n";

    cout << "\nupper BSS link check:\n";
    cout << "  node0 AP PPDUs transmitted : " << g_ap0Tx << "\n";
    cout << "  node1 STA PPDUs received   : " << g_sta1Rx << "\n";
    cout << "  node0 AP  last TX at       : " << g_ap0LastTx << " s\n";
    cout << "  node1 STA last RX at       : " << g_sta1LastRx << " s\n";
    {
        Ptr<WifiPhy> sp = DynamicCast<WifiNetDevice>(devSta1.Get(0))->GetPhy();
        Ptr<WifiPhy> ap0p = DynamicCast<WifiNetDevice>(devAp0.Get(0))->GetPhy();
        cout << "  FINAL node1 STA phy: state=" << sp->GetState()->GetState()
             << " ch=" << +sp->GetOperatingChannel().GetNumber()
             << " width=" << sp->GetOperatingChannel().GetWidth() << "\n";
        cout << "  FINAL node0 AP  phy: state=" << ap0p->GetState()->GetState()
             << " ch=" << +ap0p->GetOperatingChannel().GetNumber()
             << " width=" << ap0p->GetOperatingChannel().GetWidth() << "\n";
        if (dualRole)
        {
            Ptr<WifiPhy> ap1p = ap1->GetPhy();
            cout << "  FINAL node1 AP  phy: state=" << ap1p->GetState()->GetState()
                 << " ch=" << +ap1p->GetOperatingChannel().GetNumber() << "\n";
        }
    }
    cout << "  node1 STA PHY last state transitions:\n";
    for (size_t i = (g_stateLog.size() > 8 ? g_stateLog.size() - 8 : 0); i < g_stateLog.size(); ++i)
    {
        cout << g_stateLog[i] << "\n";
    }
    cout << "  node1 STA PHY time by state:\n";
    for (const auto& [name, secs] : g_state)
    {
        cout << "    " << setw(10) << name << " : " << secs << " s\n";
    }

    cout << "\nnode1 AP radio, PPDUs on air:\n";
    cout << "  MultiUserScheduler installed: " << (hasScheduler ? "yes" : "NO") << "\n";
    cout << "  SU PPDUs        : " << g_su << "\n";
    cout << "  DL MU (OFDMA)   : " << g_mu << "\n";
    for (const auto& [width, n] : g_muWidth)
    {
        cout << "    " << width << " STAs sharing one PPDU: " << n << "\n";
    }
    cout << "  AIDs seen in MU PPDUs: ";
    for (uint16_t a : g_aids)
    {
        cout << a << " ";
    }
    cout << "\n";

    cout << "\nforwarding through the dual-role node:\n";
    cout << "  node0 -> node2 rx: " << rx2 << " bytes\n";
    cout << "  node0 -> node3 rx: " << rx3 << " bytes\n";

    cout << "\n--- verdict ---\n";
    cout << "1. node can be AP and STA at once : " << (bothRoles ? "PASS" : "FAIL") << "\n";
    cout << "2. real MU-OFDMA on the mesh hop  : " << (g_mu > 0 ? "PASS" : "FAIL");
    if (g_mu == 0)
    {
        cout << (hasScheduler ? "  (scheduler present -> never had 2 STAs queued together)"
                              : "  (NO scheduler installed -> config error, not a real result)");
    }
    cout << "\n";
    cout << "3. multi-hop forwarding still works: " << (forwarded ? "PASS" : "FAIL") << "\n";

    Simulator::Destroy();
    return (bothRoles && g_mu > 0 && forwarded) ? 0 : 1;
}
