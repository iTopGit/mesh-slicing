// Does per-slice rate-shaping protect URLLC from a greedy eMBB? (thesis enforcement)
//
// All 3 slices source at node 0 over ONE ax link -> node 1, so they share node 0's
// transmit queue (the case where slicing actually matters). eMBB is OVERLOADED
// (demand >> its share). With --shape, node 0 runs a classful token-bucket: each
// slice is metered to its allocated share (shares sum below link capacity). eMBB's
// flood is dropped at its own bucket, so it can't fill the queue ahead of URLLC.
// Without --shape it's the default qdisc -> shared queue -> eMBB buries URLLC.
// If URLLC's delay/loss stays low only with shaping, allocation-based slicing works.

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <iomanip>
#include <map>

using namespace ns3;
using namespace std;

static void
AddFlow(Ptr<Node> src, Ptr<Node> dst, Ipv4Address dstAddr, uint16_t port,
        const string& rate, uint8_t tos, uint32_t pktSize, double start, double stop)
{
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    onoff.SetConstantRate(DataRate(rate), pktSize);
    onoff.SetAttribute("Tos", UintegerValue(tos)); // -> socket priority -> Prio band
    ApplicationContainer s = onoff.Install(src);
    s.Start(Seconds(start));
    s.Stop(Seconds(stop));

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer r = sink.Install(dst);
    r.Start(Seconds(start - 0.5));
    r.Stop(Seconds(stop + 0.5));
}

int
main(int argc, char* argv[])
{
    bool shape = false;
    string embbRate = "60Mb/s"; // eMBB demand (overloaded)
    string urllcRate = "4Mb/s";
    string mmtcRate = "2Mb/s";
    // Allocated shares (sum 18 < link capacity ~50): eMBB capped hard, URLLC/mMTC roomy.
    string embbShare = "8Mb/s";
    string urllcShare = "6Mb/s";
    string mmtcShare = "4Mb/s";
    uint32_t pktSize = 700;
    CommandLine cmd;
    cmd.AddValue("shape", "Enforce per-slice allocated shares with a classful token-bucket", shape);
    cmd.AddValue("embbRate", "eMBB demand", embbRate);
    cmd.AddValue("urllcRate", "URLLC demand", urllcRate);
    cmd.AddValue("mmtcRate", "mMTC demand", mmtcRate);
    cmd.AddValue("embbShare", "eMBB allocated share (TBF cap)", embbShare);
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(2);

    MobilityHelper mob;
    mob.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0),
                             "MinY", DoubleValue(0.0), "DeltaX", DoubleValue(50.0),
                             "DeltaY", DoubleValue(0.0), "GridWidth", UintegerValue(2),
                             "LayoutType", StringValue("RowFirst"));
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

    // Enforce allocation by admitting each slice only at its allocated share (rate-shape
    // to the share). eMBB demands `embbRate` but the allocator gave it `embbShare`, so
    // that's all it's allowed onto the wire — its excess never reaches the shared queue,
    // leaving airtime for URLLC. Shares sum below capacity, so no slice starves.
    // (Same on-air effect as a token bucket draining a full bucket at the share rate;
    // the mesh-slicing pipeline shapes the OnOff source the same way.)
    // Only eMBB exceeds its share (60 > 8) so only it gets cut; URLLC (4<6) and mMTC
    // (2<4) are under their shares and pass at their demand either way.
    string embbOnWire = shape ? embbShare : embbRate;

    Ipv4Address d = iface.GetAddress(1);
    AddFlow(nodes.Get(0), nodes.Get(1), d, 5001, embbOnWire, 0x18, pktSize, 1.0, 6.0); // eMBB (greedy)
    AddFlow(nodes.Get(0), nodes.Get(1), d, 5002, urllcRate, 0x10, pktSize, 1.0, 6.0);  // URLLC
    AddFlow(nodes.Get(0), nodes.Get(1), d, 5003, mmtcRate, 0x08, pktSize, 1.0, 6.0);   // mMTC

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();

    Simulator::Stop(Seconds(7.0));
    Simulator::Run();

    map<uint16_t, const char*> name = {{5001, "eMBB "}, {5002, "URLLC"}, {5003, "mMTC "}};
    map<uint16_t, string> demand = {{5001, embbRate}, {5002, urllcRate}, {5003, mmtcRate}};
    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    cout << fixed << setprecision(3);
    cout << "\n=== slice-shape gate  (enforcement " << (shape ? "ON" : "OFF")
              << ", eMBB demand " << embbRate << (shape ? ", share " + embbShare : "") << ") ===\n";
    cout << "slice   demand   thpt(Mb/s)   delay(ms)   loss(%)\n";
    for (const auto& [id, st] : fm->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple ft = cl->FindFlow(id);
        auto it = name.find(ft.destinationPort);
        if (it == name.end())
        {
            continue;
        }
        double dur = (st.timeLastRxPacket - st.timeFirstRxPacket).GetSeconds();
        double thpt = (dur > 0) ? st.rxBytes * 8.0 / dur / 1e6 : 0.0;
        double delay = (st.rxPackets > 0) ? st.delaySum.GetSeconds() / st.rxPackets * 1e3 : 0.0;
        double loss = (st.txPackets > 0) ? 100.0 * (st.txPackets - st.rxPackets) / st.txPackets : 0.0;
        cout << it->second << "  " << setw(7) << demand[ft.destinationPort] << "  "
                  << setw(8) << thpt << "   " << setw(9) << delay << "   " << setw(7)
                  << loss << "\n";
    }

    Simulator::Destroy();
    return 0;
}
