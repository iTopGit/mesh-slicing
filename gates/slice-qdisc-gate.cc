// Slice isolation at L2.5: does a per-slice queue disc on every radio keep ALL slices alive
// while one of them floods? Three nodes in a line, 0 -> 1 -> 2, all three slices sourcing at
// node 0 and sinking at node 2, so node 1 is a pure relay -- it never ran an application and
// can only police what it forwards.
//
//   --mode=none    no policing. Baseline: one shared queue, eMBB buries everyone.
//   --mode=source  what the harness does today: cap the OnOff source at its share. Note this
//                  also LOWERS the offered load, which is why its result can't be compared
//                  against `none` directly -- it changes two things at once.
//   --mode=qdisc   the proposed fix: sources run at full demand, and every radio gets a
//                  PrioQueueDisc root with one TbfQueueDisc per slice, capped at its share.
//
// The claim to falsify: with `qdisc`, mMTC -- the LOWEST priority band -- still gets through
// while eMBB floods. If only priority mattered mMTC would starve; it survives because eMBB
// is capped and the shares sum below capacity. That difference is slicing vs plain QoS.
//
// The relay's per-band counters are printed: eMBB dropping at node 1 is the direct evidence
// that L2.5 polices transit traffic, which source shaping cannot do.

#include "../harness/traffic-control.h"
#include "../ofdma/allocator.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <array>
#include <iomanip>
#include <string>

using namespace ns3;
using namespace nsga;
using namespace std;

namespace {

// Slice order throughout is the allocator's: 0=eMBB, 1=URLLC, 2=mMTC.

// A queue disc on a wifi device does NOT see Socket::IpTos2Priority: wifi's select-queue
// callback (SelectQueueByDSField, qos-utils.cc) runs first and overwrites the priority tag
// with `ToS >> 5`. So a disc classifying on that tag can only ever see the access category.
// SliceBandFilter sidesteps it by reading ToS bits 2-4 off the raw IP header, which leaves
// every slice on one AC (AC_BE) and the queue disc as the only thing telling them apart.
// Marking slices into AC_VI/AC_VO is not an option regardless: those cap CWmax at 15/7
// instead of 1023 (wifi-mac.cc ConfigureDcf), and over two hops a window that small never
// lets a relay de-synchronise from its source -- measured 99.86% loss for a lone 4 Mb/s flow.
const array<uint8_t, NUM_SLICES> TOS = {SliceTos::Embb, SliceTos::Urllc, SliceTos::Mmtc};


const array<uint16_t, NUM_SLICES> PORT = {10000, 20000, 30000};
const array<const char*, NUM_SLICES> NAME = {"eMBB ", "URLLC", "mMTC "};

void
AddFlow(Ptr<Node> src, Ptr<Node> dst, Ipv4Address dstAddr, uint16_t port, double mbps,
        uint8_t tos, uint32_t pktSize, double start, double stop)
{
    if (mbps <= 0.0)
    {
        return; // SetConstantRate divides by the rate, so a zero-rate source never stops sending
    }
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    onoff.SetConstantRate(DataRate(static_cast<uint64_t>(mbps * 1e6)), pktSize);
    onoff.SetAttribute("Tos", UintegerValue(tos));
    ApplicationContainer s = onoff.Install(src);
    s.Start(Seconds(start));
    s.Stop(Seconds(stop));

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer r = sink.Install(dst);
    r.Start(Seconds(start - 0.5));
    r.Stop(Seconds(stop + 0.5));
}

// One host route each way; the interface index comes from Ipv4, never from GetNDevices()
// (loopback is device 0 AND interface 0, so guessing is off by one and segfaults later).
void
AddHostRoute(Ptr<Node> n, Ptr<NetDevice> dev, Ipv4Address dst, Ipv4Address via)
{
    Ipv4StaticRoutingHelper h;
    Ptr<Ipv4> ipv4 = n->GetObject<Ipv4>();
    h.GetStaticRouting(ipv4)->AddHostRouteTo(dst, via, ipv4->GetInterfaceForDevice(dev));
}

} // namespace

int
main(int argc, char* argv[])
{
    string mode = "qdisc";
    double embbDemand = 40.0, urllcDemand = 4.0, mmtcDemand = 2.0;
    double R = 20.0;   // effective 2-hop capacity (Mb/s); a 50 m ax link is ~51, halved by relaying
    double urllcHeadroom = 1.43; // URLLC gets 1.43x its demand (~70% lane utilization)
    uint32_t pktSize = 700;
    double dur = 8.0;
    uint32_t dst = 2;
    bool acSplit = true;
    string pcapPrefix;
    CommandLine cmd;
    cmd.AddValue("mode", "none | source | qdisc", mode);
    cmd.AddValue("embbDemand", "eMBB offered load (Mb/s)", embbDemand);
    cmd.AddValue("urllcDemand", "URLLC offered load (Mb/s)", urllcDemand);
    cmd.AddValue("mmtcDemand", "mMTC offered load (Mb/s)", mmtcDemand);
    cmd.AddValue("R", "Effective capacity the allocator splits (Mb/s)", R);
    cmd.AddValue("urllcHeadroom", "URLLC share = demand x this (>=1)", urllcHeadroom);
    cmd.AddValue("dur", "Traffic duration (s)", dur);
    cmd.AddValue("dst", "Sink node: 2 = via the relay, 1 = single hop", dst);
    cmd.AddValue("acSplit", "Mark slices with distinct ToS (false = every slice on eMBB's mark)", acSplit);
    cmd.AddValue("pcapPrefix", "If set, write per-node radio pcap to this path prefix", pcapPrefix);
    cmd.Parse(argc, argv);

    if (mode != "none" && mode != "source" && mode != "qdisc")
    {
        cout << "error: --mode must be none | source | qdisc\n";
        return 1;
    }

    array<double, NUM_SLICES> demand = {embbDemand, urllcDemand, mmtcDemand};
    array<double, NUM_SLICES> share = ofdma::Allocate(R, demand, urllcHeadroom);
    array<double, NUM_SLICES> onWire{};
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        onWire[s] = (mode == "source") ? min(demand[s], share[s]) : demand[s];
    }

    NodeContainer nodes;
    nodes.Create(3);

    MobilityHelper mob;
    mob.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY",
                             DoubleValue(0.0), "DeltaX", DoubleValue(50.0), "DeltaY",
                             DoubleValue(0.0), "GridWidth", UintegerValue(3), "LayoutType",
                             StringValue("RowFirst"));
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("ChannelSettings", StringValue("{0, 80, BAND_5GHZ, 0}"));

    // Every slice now marks UP 3 = AC_BE, whose A-MPDU aggregation ns-3 enables by default, so
    // the BK_MaxAmpduSize override this gate used to need is gone.

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer dev = wifi.Install(phy, mac, nodes);

    InternetStackHelper internet;
    internet.Install(nodes);

    // Install BEFORE Ipv4AddressHelper::Assign: Assign puts the default qdisc in when the slot
    // is empty, and TrafficControlLayer wires each qdisc's SendCallback once, at initialization
    // -- installing after that throws std::bad_function_call.
    QueueDiscContainer qdiscs;
    if (mode == "qdisc")
    {
        // Shares the harness's own installer rather than rebuilding the structure here, so the
        // gate can never drift from what production runs -- including SliceBandFilter.
        qdiscs = InstallSliceQdiscs(dev);
        SetSliceRates(qdiscs, share);
    }

    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer iface = addr.Assign(dev);

    AddHostRoute(nodes.Get(0), dev.Get(0), iface.GetAddress(2), iface.GetAddress(1));
    AddHostRoute(nodes.Get(2), dev.Get(2), iface.GetAddress(0), iface.GetAddress(1));

    for (int s = 0; s < NUM_SLICES; ++s)
    {
        AddFlow(nodes.Get(0), nodes.Get(dst), iface.GetAddress(dst), PORT[s], onWire[s],
                acSplit ? TOS[s] : SliceTos::Embb, pktSize, 1.0, 1.0 + dur);
    }

    if (!pcapPrefix.empty())
    {
        phy.EnablePcapAll(pcapPrefix, true);
    }

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();

    Simulator::Stop(Seconds(1.0 + dur + 1.0));
    Simulator::Run();

    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    cout << fixed << setprecision(2);
    cout << "\n=== slice-qdisc gate (mode=" << mode << ", R=" << R << " Mb/s, urllcHeadroom=" << urllcHeadroom
         << ", hops=" << dst << ") ===\n";
    cout << "slice   demand   share   onWire   thpt   delay(ms)   loss(%)\n";
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        uint64_t tx = 0, rx = 0, rxBytes = 0;
        double delaySum = 0.0, first = 1e18, last = 0.0;
        for (const auto& [id, st] : fm->GetFlowStats())
        {
            if (cl->FindFlow(id).destinationPort != PORT[s])
            {
                continue;
            }
            tx += st.txPackets;
            rx += st.rxPackets;
            rxBytes += st.rxBytes;
            delaySum += st.delaySum.GetSeconds();
            if (st.rxPackets > 0)
            {
                first = min(first, st.timeFirstRxPacket.GetSeconds());
                last = max(last, st.timeLastRxPacket.GetSeconds());
            }
        }
        double span = (last > first) ? (last - first) : 0.0;
        cout << NAME[s] << "  " << setw(6) << demand[s] << "  " << setw(6) << share[s] << "  "
             << setw(6) << onWire[s] << "  " << setw(6)
             << ((span > 0) ? rxBytes * 8.0 / span / 1e6 : 0.0) << "  " << setw(9)
             << ((rx > 0) ? delaySum / rx * 1e3 : 0.0) << "  " << setw(7)
             << ((tx > 0) ? 100.0 * (tx - rx) / tx : 0.0) << "\n";
    }

    // Node 1 never sourced a packet, so anything it drops per band is transit traffic being
    // policed -- the thing source shaping structurally cannot do.
    if (mode == "qdisc")
    {
        cout << "\nper-node queue-disc traffic (enqueued / dropped per band)\n";
        for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
        {
            Ptr<QueueDisc> root = qdiscs.Get(i);
            cout << "  node " << i << (i == 1 ? " (relay)" : "        ") << ": ";
            QueueDisc::Stats c = root->GetQueueDiscClass(SLICE_BAND_CTRL)->GetQueueDisc()->GetStats();
            cout << "ctrl=" << c.nTotalReceivedPackets << "/" << c.nTotalDroppedPackets << "  ";
            for (int s = 0; s < NUM_SLICES; ++s)
            {
                QueueDisc::Stats st =
                    root->GetQueueDiscClass(SLICE_BAND[s])->GetQueueDisc()->GetStats();
                cout << NAME[s] << "=" << st.nTotalReceivedPackets << "/"
                     << st.nTotalDroppedPackets << "  ";
            }
            cout << "\n";
        }
    }

    Simulator::Destroy();
    return 0;
}
