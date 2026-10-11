// Gate: does ENFORCING the OFDMA split (rate-shape each slice's source to its allocated
// RU share) actually protect URLLC under congestion, and what delay does it produce? The
// allocator only decides shares on paper; the enforcer caps each source so the split is
// real (source admission = the mesh's OFDMA enforcement — ns-3 can't run real HE-OFDMA on
// a multi-hop mesh, so there is no frequency isolation, only offered-load control). Runs
// the same 3-slice shared-link scenario twice, enforcement OFF then ON, and compares. The
// measured URLLC delay under ON is what tells us which model piece 3 should feed the GA.
#include "../ofdma/allocator.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

using namespace ns3;
using namespace std;
using namespace nsga;

struct SliceResult
{
    double thpt = 0.0;
    double delay = 0.0;
    double loss = 0.0;
};

// The enforcer: install one slice's UDP source rate-shaped to `onWireMbps` (source
// admission). If the allocator gave a slice less than it demands, the excess never
// reaches the wire, freeing airtime for the others — that is the whole enforcement.
static void
InstallShapedFlow(Ptr<Node> src, Ptr<Node> dst, Ipv4Address dstAddr, uint16_t port,
                  double onWireMbps, uint32_t pktSize)
{
    ostringstream rate;
    rate << static_cast<long>(onWireMbps * 1e6) << "bps";
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    onoff.SetConstantRate(DataRate(rate.str()), pktSize);
    ApplicationContainer s = onoff.Install(src);
    s.Start(Seconds(1.0));
    s.Stop(Seconds(6.0));
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer r = sink.Install(dst);
    r.Start(Seconds(0.5));
    r.Stop(Seconds(6.5));
}

// One full scenario: 3 slices share node0->node1, enforce (shape each source to its RU
// share) or not. Returns per-slice measured {thpt, delay, loss}, 0=eMBB,1=URLLC,2=mMTC.
static array<SliceResult, NUM_SLICES>
RunScenario(const array<double, NUM_SLICES>& demand,
            const array<double, NUM_SLICES>& share,
            bool enforce,
            uint32_t pktSize)
{
    NodeContainer nodes;
    nodes.Create(2);

    MobilityHelper mob;
    mob.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY",
                             DoubleValue(0.0), "DeltaX", DoubleValue(50.0), "DeltaY",
                             DoubleValue(0.0), "GridWidth", UintegerValue(2), "LayoutType",
                             StringValue("RowFirst"));
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("ChannelSettings", StringValue("{0, 80, BAND_5GHZ, 0}"));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer dev = wifi.Install(phy, mac, nodes);

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer iface = addr.Assign(dev);

    const uint16_t port[NUM_SLICES] = {5001, 5002, 5003};
    Ipv4Address d = iface.GetAddress(1);
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        double onWire = enforce ? min(demand[s], share[s]) : demand[s];
        InstallShapedFlow(nodes.Get(0), nodes.Get(1), d, port[s], onWire, pktSize);
    }

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();
    Simulator::Stop(Seconds(7.0));
    Simulator::Run();

    array<SliceResult, NUM_SLICES> out;
    map<uint16_t, int> slot = {{5001, 0}, {5002, 1}, {5003, 2}};
    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    for (const auto& [id, st] : fm->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple ft = cl->FindFlow(id);
        auto it = slot.find(ft.destinationPort);
        if (it == slot.end())
        {
            continue;
        }
        double dur = (st.timeLastRxPacket - st.timeFirstRxPacket).GetSeconds();
        SliceResult r;
        r.thpt = (dur > 0) ? st.rxBytes * 8.0 / dur / 1e6 : 0.0;
        r.delay = (st.rxPackets > 0) ? st.delaySum.GetSeconds() / st.rxPackets * 1e3 : 0.0;
        r.loss = (st.txPackets > 0) ? 100.0 * (st.txPackets - st.rxPackets) / st.txPackets : 0.0;
        out[it->second] = r;
    }
    Simulator::Destroy();
    return out;
}

static void
PrintTable(const char* label,
           const array<double, NUM_SLICES>& demand,
           const array<double, NUM_SLICES>& share,
           bool enforce,
           const array<SliceResult, NUM_SLICES>& r)
{
    const char* nm[NUM_SLICES] = {"eMBB ", "URLLC", "mMTC "};
    cout << "\n" << label << "\n";
    cout << "  slice   demand   onWire   thpt(Mb/s)   delay(ms)   loss(%)\n";
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        double onWire = enforce ? min(demand[s], share[s]) : demand[s];
        cout << "  " << nm[s] << "   " << setw(5) << demand[s] << "   " << setw(6) << onWire << "   "
             << setw(8) << r[s].thpt << "     " << setw(8) << r[s].delay << "    " << setw(6)
             << r[s].loss << "\n";
    }
}

int
main(int argc, char* argv[])
{
    double embb = 60.0, urllc = 10.0, mmtc = 4.0, R = 51.0, urllcHeadroom = 2.0, C = 4.53;
    uint32_t pktSize = 700;
    CommandLine cmd;
    cmd.AddValue("embb", "eMBB demand Mb/s", embb);
    cmd.AddValue("urllc", "URLLC demand Mb/s", urllc);
    cmd.AddValue("mmtc", "mMTC demand Mb/s", mmtc);
    cmd.AddValue("R", "Link capacity for the allocator Mb/s", R);
    cmd.AddValue("urllcHeadroom", "URLLC share = demand x this (>=1)", urllcHeadroom);
    cmd.AddValue("C", "Model delay constant ms", C);
    cmd.AddValue("pktSize", "UDP payload bytes", pktSize);
    cmd.Parse(argc, argv);

    array<double, NUM_SLICES> demand = {embb, urllc, mmtc};
    array<double, NUM_SLICES> share = ofdma::Allocate(R, demand, urllcHeadroom);

    cout << fixed << setprecision(3);
    cout << "=== OFDMA enforce gate (3 slices, one ax link) ===\n";
    cout << "demand  eMBB=" << embb << "  URLLC=" << urllc << "  mMTC=" << mmtc << "  (R=" << R
         << ")\n";
    cout << "OFDMA RU share  eMBB=" << share[0] << "  URLLC=" << share[1] << "  mMTC=" << share[2]
         << "   (urllcHeadroom=" << urllcHeadroom << ")\n";

    array<SliceResult, NUM_SLICES> off = RunScenario(demand, share, false, pktSize);
    PrintTable("enforcement OFF (sources at full demand):", demand, share, false, off);
    array<SliceResult, NUM_SLICES> on = RunScenario(demand, share, true, pktSize);
    PrintTable("enforcement ON (sources shaped to RU share):", demand, share, true, on);

    // Headroom H puts URLLC's lane at utilization 1/H, so the isolation model is C/(1-1/H).
    double predictIso = C / (1.0 - 1.0 / urllcHeadroom);
    double offered = min(demand[0], share[0]) + min(demand[1], share[1]) + min(demand[2], share[2]);
    double uAgg = offered / R;
    double predictShared = (uAgg < 1.0) ? C / (1.0 - uAgg) : INFINITY; // shared-channel model

    cout << "\nURLLC delay:  OFF=" << off[1].delay << " ms  ->  ON=" << on[1].delay << " ms\n";
    cout << "candidate models for ON:  per-RU isolation C/(1-1/H)=" << predictIso
         << " ms   shared-channel C/(1-Uagg)=" << predictShared << " ms  (Uagg=" << uAgg << ")\n";

    bool protectedOk = on[1].delay < off[1].delay;
    bool servedOk = on[1].thpt >= urllc * 0.9;
    cout << "\nenforcing lowers URLLC delay: " << (protectedOk ? "PASS" : "FAIL") << " (OFF "
         << off[1].delay << " -> ON " << on[1].delay << " ms)\n";
    cout << "URLLC still served (thpt >= 90% demand): " << (servedOk ? "PASS" : "FAIL") << " ("
         << on[1].thpt << " Mb/s)\n";
    return (protectedOk && servedOk) ? 0 : 1;
}
