// Gate: can eMBB's cap be FOUND by measurement instead of computed from a capacity model?
//
// `ofdma/enforcement.cc` derives R analytically (link rate / average hops) and gets it badly wrong
// on the 5x5 harness -- 18.38 Mb/s modelled against 11.02 actually delivered. Caps built on that R
// sit above what the network can carry, so they never bite and the slices congest anyway.
//
// Alloc.md's answer is not to repair the model but to delete it from this path. Only ONE cap needs
// a capacity figure at all: URLLC's is `demand * urllcHeadroom` and mMTC's is its demand, both
// built from quantities that are known, small and inelastic (both source papers characterise those
// two classes that way; see Alloc.md). eMBB is the elastic residual, and a quantity defined as
// "whatever is spare" should be walked to, not calculated.
//
// So this gate runs AIMD on eMBB's cap alone -- additive increase, multiplicative decrease, the
// TCP control law -- driven by a local congestion signal.
//
// THE SIGNAL IS THE WHOLE POINT, and two wrong choices were already made here:
//   * v1 sensed packet LOSS on offered traffic. That counts the disc's own policing as congestion,
//     so the loop chased its own tail to zero (25% -> 91% loss in one run).
//   * v2 sensed loss on ADMITTED traffic. Better, but after every slice moved to AC_BE the chain
//     began absorbing congestion as queueing DELAY rather than drops -- measured loss read
//     0.00-0.65% while delay hit 453 ms, so the upward probe never stopped (100 -> 71.5, climbing).
//   * v3, here, senses QUEUE DEPTH of the PROTECTED slices only.
//
// Why protected-only: URLLC and mMTC are never over their own caps, because their caps are set to
// (at least) their demands. So if THEIR queues build, it is not policing -- the radio genuinely
// cannot drain them, which is the medium saying it is full. eMBB's own queue is deliberately NOT
// read: it floods by design and is permanently backlogged, and reading it would repeat exactly the
// mistake of v1.
//
//   --mode=fixed     caps frozen, eMBB capped from a supplied R via ofdma::Allocate. Reproduces
//                    the harness failure: a cap above real capacity never bites.
//   --mode=feedback  eMBB's cap found by AIMD on protected-slice queue depth. No R anywhere.
//
// The claim to falsify: feedback protects URLLC and mMTC without ever being told the capacity.

#include "../harness/traffic-control.h" // SliceTos, InstallSliceQdiscs, SetSliceRates
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

const array<uint8_t, NUM_SLICES> TOS = {SliceTos::Embb, SliceTos::Urllc, SliceTos::Mmtc};
const array<uint16_t, NUM_SLICES> PORT = {10000, 20000, 30000};
const array<const char*, NUM_SLICES> NAME = {"eMBB ", "URLLC", "mMTC "};

// Congestion probe: how many packets are waiting in the RADIO's own transmit queue.
//
// Three sensors were tried at the queue-disc layer first. All three fail, and the reason is the
// same each time, so it is worth recording:
//   * protected-slice queue DEPTH, mMTC capped at exactly its demand -- its lane sits at
//     utilization 1 and its queue grows without bound by itself. Backlog passed 80 packets while
//     eMBB was throttled to 0.10 Mb/s, with the medium nearly idle. It was reading mMTC policing
//     itself.
//   * the same, once both protected slices had headroom -- their queues drain freely, depth reads
//     0 forever, and eMBB's cap grows unbounded.
//   * protected-slice SOJOURN time -- reads 0.00 ms always, because `Peek()` finds those queues
//     EMPTY at every sample.
//
// That last result is the diagnostic one. The token buckets admit protected traffic immediately,
// so nothing accumulates at L2.5; URLLC's measured 25.91 ms is spent BELOW the queue disc, in the
// MAC waiting for the medium. Congestion is simply not visible from the queue disc, and no
// cleverness about which queue-disc statistic to read will make it visible.
//
// So read the device. Every slice now marks UP 3 = AC_BE, so one MAC queue carries all of it, and
// its depth is the radio saying whether it can keep up. That is medium congestion directly, not a
// proxy, and it cannot be confused with a slice's own policing because the buckets sit upstream
// of it.
uint32_t
RadioBacklog(const NetDeviceContainer& devices)
{
    uint32_t worst = 0;
    for (uint32_t i = 0; i < devices.GetN(); ++i)
    {
        Ptr<WifiNetDevice> wifi = DynamicCast<WifiNetDevice>(devices.Get(i));
        if (!wifi)
        {
            continue;
        }
        Ptr<WifiMacQueue> q = wifi->GetMac()->GetTxopQueue(AC_BE);
        if (q)
        {
            worst = std::max(worst, q->GetNPackets());
        }
    }
    return worst;
}

struct Controller
{
    QueueDiscContainer qdiscs;
    NetDeviceContainer devices;
    std::array<double, NUM_SLICES> demand{};
    double urllcHeadroom = 2.0;
    double period = 0.5;

    double embbCap = 0.0;    // the ONE quantity under control (Mb/s)
    double embbCapMax = 0.0; // eMBB's own demand: a cap above what it offers does nothing
    double embbCapMin = 0.1;

    uint32_t radioTargetPkts = 10; // above this the radio is behind, not just busy
    double aiStep = 1.0;          // additive increase per tick (Mb/s)
    double mdFactor = 0.5;        // multiplicative decrease -- TCP's value
    uint32_t lastBacklog = 0;

    bool verbose = true;
};

// URLLC and mMTC never move. Only eMBB's cap is controlled.
//
// BOTH reserved slices get headroom, not just URLLC. Setting mMTC's cap to exactly its demand puts
// its lane at utilization 1, where queueing delay grows without bound -- and measured, it does:
// mMTC's backlog climbed past 80 packets and its delay hit 270 ms while eMBB was throttled to
// 0.10 Mb/s, i.e. with the medium nearly empty. That is mMTC policing itself, and reading it as
// medium congestion is the same mistake the loss-based versions made. The queueing formula does
// not care which slice it is applied to.
void
ApplyCaps(Controller* c)
{
    std::array<double, NUM_SLICES> share = {c->embbCap,
                                            c->demand[1] * c->urllcHeadroom,
                                            c->demand[2] * c->urllcHeadroom};
    SetSliceRates(c->qdiscs, share);
}

void
Tick(Controller* c)
{
    uint32_t backlog = RadioBacklog(c->devices);
    bool over = backlog > c->radioTargetPkts;
    bool draining = backlog < c->lastBacklog;

    // Three states, not two. A queue takes time to drain, so cutting again while the previous cut
    // is still working over-corrects: the first version of this loop climbed to 32 Mb/s, then cut
    // five ticks in a row down to 1 Mb/s before the backlog had finished responding. TCP has the
    // same guard -- one reduction per round trip, not one per acknowledgement. HOLD needs no new
    // constant: a falling backlog is itself the evidence that the last cut was enough.
    const char* action;
    if (over && !draining)
    {
        c->embbCap *= c->mdFactor;
        action = "CONGESTED";
    }
    else if (over)
    {
        action = "draining ";
    }
    else
    {
        c->embbCap += c->aiStep;
        action = "  clear  ";
    }
    c->lastBacklog = backlog;
    c->embbCap = std::clamp(c->embbCap, c->embbCapMin, c->embbCapMax);
    ApplyCaps(c);

    if (c->verbose)
    {
        cout << "  t=" << fixed << setprecision(1) << Simulator::Now().GetSeconds()
             << "s  radioBacklog=" << setw(4) << backlog << " pkt  "
             << action << "  ->  eMBB cap=" << setprecision(2)
             << setw(6) << c->embbCap << " Mb/s\n";
    }
    Simulator::Schedule(Seconds(c->period), &Tick, c);
}

void
AddFlow(Ptr<Node> src, Ptr<Node> dst, Ipv4Address dstAddr, uint16_t port, double mbps,
        uint8_t tos, uint32_t pktSize, double start, double stop)
{
    if (mbps <= 0.0)
    {
        return;
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
    string mode = "feedback";
    double embbDemand = 40.0, urllcDemand = 4.0, mmtcDemand = 2.0;
    double r0 = 40.0; // deliberately too high: this is the harness's failure mode
    double urllcHeadroom = 2.0;
    double period = 0.5;
    double aiStep = 1.0;
    double mdFactor = 0.5;
    uint32_t radioTarget = 10;
    uint32_t pktSize = 1472;
    double dur = 10.0;
    uint32_t nodes_n = 3;
    CommandLine cmd;
    cmd.AddValue("mode", "fixed | feedback", mode);
    cmd.AddValue("embbDemand", "eMBB offered load (Mb/s)", embbDemand);
    cmd.AddValue("urllcDemand", "URLLC offered load (Mb/s)", urllcDemand);
    cmd.AddValue("mmtcDemand", "mMTC offered load (Mb/s)", mmtcDemand);
    cmd.AddValue("r0", "--mode=fixed only: the capacity estimate fed to ofdma::Allocate", r0);
    cmd.AddValue("aiStep", "AIMD additive increase per tick (Mb/s)", aiStep);
    cmd.AddValue("mdFactor", "AIMD multiplicative decrease (TCP uses 0.5)", mdFactor);
    cmd.AddValue("radioTarget",
                 "Wifi MAC queue depth (packets) above which the medium counts as congested",
                 radioTarget);
    cmd.AddValue("urllcHeadroom", "URLLC share = demand x this (>=1)", urllcHeadroom);
    cmd.AddValue("period", "Controller period (s)", period);
    cmd.AddValue("dur", "Traffic duration (s)", dur);
    cmd.Parse(argc, argv);

    if (mode != "fixed" && mode != "feedback")
    {
        cout << "error: --mode must be fixed | feedback\n";
        return 1;
    }

    NodeContainer nodes;
    nodes.Create(nodes_n);

    MobilityHelper mob;
    mob.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY",
                             DoubleValue(0.0), "DeltaX", DoubleValue(50.0), "DeltaY",
                             DoubleValue(0.0), "GridWidth", UintegerValue(nodes_n), "LayoutType",
                             StringValue("RowFirst"));
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("ChannelSettings", StringValue("{0, 80, BAND_5GHZ, 0}"));
    phy.Set("TxPowerStart", DoubleValue(20.0)); // match harness/topology.cc
    phy.Set("TxPowerEnd", DoubleValue(20.0));
    // Carrier sense left at the ns-3 default: on a 3-node chain the harness's -86 would stop
    // node 0 hearing node 2, adding a hidden terminal that has nothing to do with the controller.
    // Every slice marks UP 3 = AC_BE now, whose A-MPDU aggregation ns-3 enables by default,
    // so the BK_MaxAmpduSize override this gate used to need is gone.

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer dev = wifi.Install(phy, mac, nodes);

    InternetStackHelper internet;
    internet.Install(nodes);

    QueueDiscContainer qdiscs = InstallSliceQdiscs(dev); // before Assign, see traffic-control.h

    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer iface = addr.Assign(dev);

    AddHostRoute(nodes.Get(0), dev.Get(0), iface.GetAddress(2), iface.GetAddress(1));
    AddHostRoute(nodes.Get(2), dev.Get(2), iface.GetAddress(0), iface.GetAddress(1));

    array<double, NUM_SLICES> demand = {embbDemand, urllcDemand, mmtcDemand};
    // Sources always offer their full demand; the caps do the policing, so the two arms differ
    // only in whether the cap ever gets corrected.
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        AddFlow(nodes.Get(0), nodes.Get(2), iface.GetAddress(2), PORT[s], demand[s], TOS[s],
                pktSize, 1.0, 1.0 + dur);
    }

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();
    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());

    // --mode=fixed reproduces the harness: every cap comes out of a capacity estimate, including
    // eMBB's, and if that estimate is too high the caps never bite. --mode=feedback never uses R
    // at all -- URLLC and mMTC are set from their own demands and eMBB is walked to by AIMD.
    Controller ctrl;
    ctrl.qdiscs = qdiscs;
    ctrl.demand = demand;
    ctrl.urllcHeadroom = urllcHeadroom;
    ctrl.period = period;
    ctrl.aiStep = aiStep;
    ctrl.mdFactor = mdFactor;
    ctrl.radioTargetPkts = radioTarget;
    ctrl.devices = dev;
    ctrl.embbCapMax = demand[0]; // a cap above eMBB's own offered load does nothing
    ctrl.embbCap = demand[0];    // start uncapped, so the trace shows it walking DOWN

    cout << fixed << setprecision(2);
    cout << "\n=== capacity-feedback gate (mode=" << mode << ", urllcHeadroom=" << urllcHeadroom
         << ") ===\n";

    if (mode == "fixed")
    {
        array<double, NUM_SLICES> share0 = ofdma::Allocate(r0, demand, urllcHeadroom);
        SetSliceRates(qdiscs, share0);
        cout << "R=" << r0 << " Mb/s (frozen) -> caps [eMBB/URLLC/mMTC] = " << share0[0] << "/"
             << share0[1] << "/" << share0[2] << " Mb/s\n";
    }
    else
    {
        ApplyCaps(&ctrl);
        cout << "no R anywhere. URLLC=" << (demand[1] * urllcHeadroom) << " mMTC=" << demand[2]
             << " fixed from demand; eMBB starts at " << ctrl.embbCap
             << " and is found by AIMD (+" << aiStep << " / x" << mdFactor << ", congested above "
             << radioTarget << " pkt in the radio)\ncontroller trace:\n";
        Simulator::Schedule(Seconds(1.0 + period), &Tick, &ctrl);
    }

    Simulator::Stop(Seconds(1.0 + dur + 1.0));
    Simulator::Run();

    cout << "\nslice   demand   thpt   delay(ms)   loss(%)\n";
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        uint64_t tx = 0, rx = 0, rxBytes = 0;
        double delaySum = 0.0, first = 1e18, lastT = 0.0;
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
                lastT = max(lastT, st.timeLastRxPacket.GetSeconds());
            }
        }
        double span = (lastT > first) ? (lastT - first) : 0.0;
        cout << NAME[s] << "  " << setw(6) << demand[s] << "  " << setw(6)
             << ((span > 0) ? rxBytes * 8.0 / span / 1e6 : 0.0) << "  " << setw(9)
             << ((rx > 0) ? delaySum / rx * 1e3 : 0.0) << "  " << setw(7)
             << ((tx > 0) ? 100.0 * (tx - rx) / tx : 0.0) << "\n";
    }
    if (mode == "feedback")
    {
        cout << "\nfinal eMBB cap: " << ctrl.embbCap << " Mb/s (started at " << demand[0]
             << ", never told the capacity)\n";
    }
    PrintSliceQdiscStats(qdiscs);

    Simulator::Destroy();
    return 0;
}
