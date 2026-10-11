// Gate: the four-step allocation rule of Alloc.md 4c.2, exercised through the production
// allocator. This gate holds NO copy of the rule -- it calls ofdma::Allocate, so it cannot
// drift from what the harness runs.
//
//   1. cap_1 = min( lambda_1 * Headroom,  rho* * C )          URLLC, served first
//   2. Cleft = C - cap_1
//   3. cap_2 = min( lambda_2 * mmtcHeadroom, (1-F) * Cleft )  mMTC
//   4. cap_3 = Cleft - cap_2                                  eMBB, residual
//
// C is FOUND, never computed: AIMD on the radio's own queue depth. No capacity model, no
// EffectiveCapacity, no hop division, no reuse factor.
//
// What changed from the previous version of this gate, and why it matters: AIMD used to tune
// `X`, a budget shared by eMBB and mMTC only, with URLLC's cap fixed at demand * H and bounded
// by nothing. Raising --urllcDemand then let URLLC take 39.79 Mb/s while eMBB got 1.46 and mMTC
// 0.53 -- AIMD correctly cut X, but the cut landed entirely on the innocent slices. A
// demand-derived reservation is a blank cheque. AIMD now tunes C, the whole medium's capacity,
// so URLLC's rho* * C ceiling descends with everything else.
//
// THE CAP IS A CONTENTION REFEREE, NOT A LIMIT. While the medium is quiet C grows and every cap
// floats ABOVE the slices' demands, so nobody is policed; when the medium fills C descends until
// the caps bind and F decides how the squeeze is shared. A cap that bites while capacity is spare
// is pure waste, and `--mode=clamped` reproduces exactly that bug: C pinned to eMBB's own demand,
// which starves mMTC on a half-empty medium (mMTC 10 Mb/s + eMBB 2 Mb/s on a 15 Mb/s link -> the
// radio never congests, AIMD stalls at the clamp, mMTC is held to 2).

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

#include <algorithm>
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

// Congestion signal: the radio's own transmit queue. Every slice marks AC_BE, so one MAC queue
// carries all of it. Read the device, not the queue disc -- the buckets admit protected traffic
// immediately, so L2.5 queues are found EMPTY at every sample even while end-to-end delay is
// hundreds of ms. The waiting happens below, in the MAC. (Four queue-disc-level sensors were tried
// first; see the capacity-feedback-gate header.)
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
            worst = max(worst, q->GetNPackets());
        }
    }
    return worst;
}

struct Alloc
{
    QueueDiscContainer qdiscs;
    NetDeviceContainer devices;
    array<double, NUM_SLICES> demand{};

    double urllcHeadroom = 1.64; // 1/(1 - T0/T*)
    double embbFloor = 0.3;      // F, the design's one policy number
    double mmtcHeadroom = 1.0;

    double capacity = 0.0; // the AIMD-found mesh capacity C -- the ONLY thing AIMD moves
    double capMax = 0.0;   // physical ceiling; must NOT be eMBB's demand (see header)
    double capMin = 0.2;

    uint32_t radioTargetPkts = 10;
    double aiStep = 1.0;
    double mdFactor = 0.5;
    uint32_t lastBacklog = 0;

    bool verbose = true;
};

// The whole rule, in one call. No copy of the arithmetic lives in this gate.
array<double, NUM_SLICES>
Caps(const Alloc& a)
{
    return ofdma::Allocate(a.capacity, a.demand, a.urllcHeadroom, a.embbFloor, a.mmtcHeadroom);
}

void
ApplyCaps(Alloc* a)
{
    SetSliceRates(a->qdiscs, Caps(*a));
}

void
Tick(Alloc* a)
{
    uint32_t backlog = RadioBacklog(a->devices);
    bool over = backlog > a->radioTargetPkts;
    bool draining = backlog < a->lastBacklog;

    // Three states. A queue takes time to drain, so cutting again while the previous cut is still
    // working over-corrects. A falling backlog is itself the evidence that the last cut was enough,
    // so HOLD needs no extra constant.
    const char* action;
    if (over && !draining)
    {
        a->capacity *= a->mdFactor;
        action = "CONGESTED";
    }
    else if (over)
    {
        action = "draining ";
    }
    else
    {
        a->capacity += a->aiStep;
        action = "  clear  ";
    }
    a->lastBacklog = backlog;
    a->capacity = clamp(a->capacity, a->capMin, a->capMax);
    ApplyCaps(a);

    if (a->verbose)
    {
        array<double, NUM_SLICES> c = Caps(*a);
        cout << "  t=" << fixed << setprecision(1) << Simulator::Now().GetSeconds()
             << "s  radioQ=" << setw(4) << backlog << " pkt  " << action << setprecision(2)
             << "  C=" << setw(6) << a->capacity << "  caps e/u/m=" << c[0] << "/" << c[1] << "/"
             << c[2] << "\n";
    }
    Simulator::Schedule(Seconds(0.5), &Tick, a);
}

void
AddFlow(Ptr<Node> src, Ptr<Node> dst, Ipv4Address dstAddr, uint16_t port, double mbps, uint8_t tos,
        uint32_t pktSize, double start, double stop)
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
    string mode = "adaptive";
    double embbDemand = 40.0, urllcDemand = 4.0, mmtcDemand = 2.0;
    double urllcTargetMs = 10.0; // T*, IEEE 802.11 RTA TIG via Khorov et al., arXiv:2107.03541
    double baseDelayMs = 3.9;    // T0, formula-gate: 7 loads x 3 seeds, Alloc.md 0.6
    double embbFloor = 0.3;      // F -- POLICY, the only chosen number. Sweep it (Alloc.md 4c.7)
    double mmtcHeadroom = 1.0;   // constant-rate source cannot overshoot its rate (Alloc.md 4c.5)
    double linkRateMbps = 80.0;  // measured median; ceiling on C, NOT a capacity estimate
    double aiStep = 1.0, mdFactor = 0.5;
    uint32_t radioTarget = 10, pktSize = 1472;
    double dur = 20.0;

    CommandLine cmd;
    cmd.AddValue("mode", "adaptive | clamped (clamped pins C to eMBB's demand: the bug)", mode);
    cmd.AddValue("embbDemand", "eMBB offered load (Mb/s)", embbDemand);
    cmd.AddValue("urllcDemand", "URLLC offered load (Mb/s)", urllcDemand);
    cmd.AddValue("mmtcDemand", "mMTC offered load (Mb/s)", mmtcDemand);
    cmd.AddValue("urllcTargetMs", "URLLC delay target T* (ms)", urllcTargetMs);
    cmd.AddValue("baseDelayMs", "Calibration constant T0 (ms)", baseDelayMs);
    cmd.AddValue("embbFloor", "F: eMBB's guaranteed share of what URLLC leaves", embbFloor);
    cmd.AddValue("mmtcHeadroom", "mMTC share = demand x this (1.0 = no headroom)", mmtcHeadroom);
    cmd.AddValue("linkRate", "Physical ceiling on C (Mb/s)", linkRateMbps);
    cmd.AddValue("aiStep", "AIMD additive increase per tick (Mb/s)", aiStep);
    cmd.AddValue("mdFactor", "AIMD multiplicative decrease", mdFactor);
    cmd.AddValue("radioTarget", "Radio queue depth counted as congested (packets)", radioTarget);
    cmd.AddValue("dur", "Traffic duration (s)", dur);
    cmd.Parse(argc, argv);

    // T* <= T0 is not a tight target, it is an impossible one: even an empty queue costs T0.
    if (urllcTargetMs <= baseDelayMs)
    {
        cout << "error: urllcTargetMs (" << urllcTargetMs << ") must exceed baseDelayMs ("
             << baseDelayMs << ") -- even an empty queue takes T0, so the target is unreachable\n";
        return 1;
    }
    double headroom = urllcTargetMs / (urllcTargetMs - baseDelayMs);

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
    phy.Set("TxPowerStart", DoubleValue(20.0)); // match harness/topology.cc
    phy.Set("TxPowerEnd", DoubleValue(20.0));
    // Carrier sense left at the ns-3 default: on a 3-node chain the harness's -86 would stop node 0
    // hearing node 2, adding a hidden terminal unrelated to the allocator.

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
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        AddFlow(nodes.Get(0), nodes.Get(2), iface.GetAddress(2), PORT[s], demand[s], TOS[s],
                pktSize, 1.0, 1.0 + dur);
    }

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();
    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());

    Alloc a;
    a.qdiscs = qdiscs;
    a.devices = dev;
    a.demand = demand;
    a.urllcHeadroom = headroom;
    a.embbFloor = embbFloor;
    a.mmtcHeadroom = mmtcHeadroom;
    a.aiStep = aiStep;
    a.mdFactor = mdFactor;
    a.radioTargetPkts = radioTarget;
    // The clamped arm reproduces the bug: C pinned to eMBB's own demand means AIMD stalls whenever
    // eMBB is not the greedy slice, dragging the other caps down with it.
    a.capMax = (mode == "clamped") ? embbDemand : linkRateMbps;
    a.capacity = a.capMax;

    array<double, NUM_SLICES> c0 = Caps(a);
    cout << fixed << setprecision(2);
    cout << "\n=== alloc gate (mode=" << mode << ") ===\n"
         << "URLLC: T*=" << urllcTargetMs << " ms, T0=" << baseDelayMs
         << " ms -> Headroom=" << headroom << ", rho*=" << 1.0 / headroom << "\n"
         << "F=" << embbFloor << " (policy), mmtcHeadroom=" << mmtcHeadroom << "\n"
         << "C starts " << a.capacity << ", ceiling " << a.capMax << " Mb/s\n"
         << "demand e/u/m = " << embbDemand << "/" << urllcDemand << "/" << mmtcDemand
         << "   initial caps e/u/m = " << c0[0] << "/" << c0[1] << "/" << c0[2] << "\n\n";

    ApplyCaps(&a);
    Simulator::Schedule(Seconds(1.5), &Tick, &a);
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
    array<double, NUM_SLICES> cEnd = Caps(a);
    cout << "\nfinal C = " << a.capacity << " Mb/s (never computed from a capacity model)"
         << "   caps e/u/m = " << cEnd[0] << "/" << cEnd[1] << "/" << cEnd[2] << "\n";
    PrintSliceQdiscStats(qdiscs);

    // The work-conserving check: policing a slice while the radio was never congested means the cap
    // bit with capacity to spare, which is the failure mode --mode=clamped exists to demonstrate.
    cout << "\nwork-conserving check (policing with spare capacity is a FAILURE):\n";
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        QueueDisc::Stats st =
            qdiscs.Get(0)->GetQueueDiscClass(SLICE_BAND[s])->GetQueueDisc()->GetStats();
        double pct = st.nTotalReceivedPackets > 0
                         ? 100.0 * st.nTotalDroppedPackets / st.nTotalReceivedPackets
                         : 0.0;
        cout << "  " << NAME[s] << " policed " << setw(6) << pct << "%"
             << (pct > 1.0 ? "   <-- check whether the medium was actually full" : "") << "\n";
    }

    Simulator::Destroy();
    return 0;
}
