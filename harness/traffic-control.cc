#include "traffic-control.h"

#include "ns3/data-rate.h"
#include "ns3/ipv4-queue-disc-item.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/packet-filter.h"
#include "ns3/queue-disc.h"
#include "ns3/string.h"
#include "ns3/traffic-control-helper.h"
#include "ns3/uinteger.h"

#include <iostream>
#include <vector>

using namespace ns3;
using namespace nsga;
using namespace std;

namespace ns3
{

// Classifies a packet into its slice's PrioQueueDisc band by reading the slice ID out of ToS
// bits 2-4. PrioQueueDisc uses a filter's return value directly AS the band index.
//
// It must read the RAW header byte, never the SocketPriorityTag: by the time a disc runs,
// wifi's SelectQueueByDSField has already replaced that tag with `ToS >> 5`, i.e. the access
// category, which is exactly the coupling this filter exists to break.
class SliceBandFilter : public PacketFilter
{
  public:
    static TypeId GetTypeId();

  private:
    bool CheckProtocol(Ptr<QueueDiscItem> item) const override;
    int32_t DoClassify(Ptr<QueueDiscItem> item) const override;
};

NS_OBJECT_ENSURE_REGISTERED(SliceBandFilter);

TypeId
SliceBandFilter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SliceBandFilter")
                            .SetParent<PacketFilter>()
                            .SetGroupName("TrafficControl")
                            .AddConstructor<SliceBandFilter>();
    return tid;
}

bool
SliceBandFilter::CheckProtocol(Ptr<QueueDiscItem> item) const
{
    // ARP carries no DS field and returns false here, falling through to the all-zero Priomap.
    return DynamicCast<Ipv4QueueDiscItem>(item) != nullptr;
}

int32_t
SliceBandFilter::DoClassify(Ptr<QueueDiscItem> item) const
{
    uint8_t tos = 0;
    if (!item->GetUint8Value(QueueItem::IP_DSFIELD, tos))
    {
        return PacketFilter::PF_NO_MATCH;
    }
    return SLICE_ID_BAND[SliceIdFromTos(tos)];
}

} // namespace ns3

namespace {

// Every band decision now comes from SliceBandFilter, so the priomap only ever sees what the
// filter declined (ARP). All-zero sends that to the uncapped control band.
const char* PRIOMAP = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0";

// Burst is the one-MTU floor RFC 2697 forces: below the largest possible packet, a full-size
// packet can never accumulate enough tokens and waits forever. There is no freedom in it, which
// is the point -- it replaces the 15000/60000 chosen here with no source, whose implied delay
// budgets (15 ms URLLC, 57 ms eMBB, 240 ms mMTC) nobody ever decided. ns-3's own 125 kB default
// would let a slice dump ~1 Mb at once, the exact latency spike URLLC must not see.
//
// Real devices do carry fatter buckets, but for a reason that does not apply here: Linux `tc tbf`
// needs burst >= rate/HZ because the kernel refills tokens on timer ticks. ns-3 accrues from
// elapsed time on demand (tbf-queue-disc.cc:202), so there is no tick to cover. See Alloc.md 4.6.
const uint32_t MTU_BYTES = 1500;

// A TbfQueueDisc's Rate is set-only, so the installed cap cannot be read back off the disc; it
// is recorded here as it is written. All three slices, because "is the cap binding?" is a
// question about every slice, not only URLLC.
vector<array<double, NUM_SLICES>> g_lastLimitMbps;

// All three slices keep ns-3's default depth (1000p) by simply not being given one. "Unchanged
// from the simulator's default" is a stronger defence than any number chosen here, and it replaces
// the invented 200p.
//
// Equal across slices since 2026-10-13 (user's choice of the two honest options). Depth decides
// which SYMPTOM congestion produces - a deep bucket makes a burst wait, a shallow one throws it
// away - so URLLC on a derived 13 against eMBB and mMTC on 1000 made their loss figures products
// of the buffer rather than of the allocation. The claim under test ("URLLC degrades last") is
// about RATE, so rate is what may differ between slices and depth is what must not. The cost is
// named rather than hidden: at URLLC's measured per-node caps 1000p is 1.3-6.7 s of buffer against
// a 10 ms target, so its congestion now shows up as delay instead of loss. The alternative - the
// derived K for all three - would have borrowed URLLC's PLR target for two slices that have none,
// which is the borrowing that killed 1.64 and 24.
//
// Measured both ways (alloc-gate baseline, 2026-10-05) before settling on the default: 1000p vs
// 200p moves eMBB's own delay (655 vs 219 ms) and NOTHING else -- C settles at 39.75 either way,
// URLLC holds 3.65 ms, mMTC 244.6 ms. The depth never reaches the controller, because the token
// bucket's output rate is the same whatever is queued behind it. So the cost of the default falls
// entirely on the one slice with no delay requirement, which is where it belongs.

// Smallest cap a token bucket may be given. DERIVED, not chosen: a bucket must pass at least one
// maximum-size packet between controller retunes, or it cannot make progress before the rate is
// looked at again, and its backlog is effectively frozen. So the floor is one MTU per control
// period -- 1500 B x 8 / 0.5 s = 24 kb/s = 0.024 Mb/s at the default period, and it FOLLOWS
// --ctrlPeriod rather than being frozen at that value.
//
// It replaced a typed-in 0.01 Mb/s on 2026-10-14. That value was 2.4x slower and measurably too
// slow: at 13 Mb/s a bucket on the old floor still held 91 packets when the 120 s drain cap hit,
// because a 728 B packet needs 0.58 s to clear at 0.01 Mb/s.
const double CTRL_PERIOD_DEFAULT_SEC = 0.5;
double g_minLimitMbps = MTU_BYTES * 8.0 / CTRL_PERIOD_DEFAULT_SEC / 1e6;

// The Tbf's child discipline. Cobalt by default, for the reasons in InstallSliceQdiscs; Fifo is
// what a Tbf builds for itself, so it is the arm carrying no latency policy at all. Both are
// peek-safe, which FqCoDel and CoDel are not -- that list is the hard constraint here, not taste.
std::string g_sliceAqm = "ns3::CobaltQueueDisc";

} // namespace

QueueDiscContainer
InstallSliceQdiscs(const NetDeviceContainer& devices)
{
    TrafficControlHelper tch;
    uint16_t h = tch.SetRootQueueDisc("ns3::PrioQueueDisc", "Priomap", StringValue(PRIOMAP));
    tch.AddPacketFilter(h, "ns3::SliceBandFilter");
    TrafficControlHelper::ClassIdList cls = tch.AddQueueDiscClasses(h, 4, "ns3::QueueDiscClass");
    tch.AddChildQueueDisc(h, cls[SLICE_BAND_CTRL], "ns3::FifoQueueDisc");
    // MaxSize deliberately not given on any slice band, so all three keep ns-3's 1000p default.
    //
    // Each Tbf gets an EXPLICIT FqCoDel child instead of the Fifo it would create for itself
    // (tbf-queue-disc.cc:302). The Tbf still owns the slice's RATE; the child owns the slice's
    // LATENCY, and the two are different jobs. With the default Fifo all 12 flows of a slice share
    // one 1000-packet line that has no packet lifetime and nothing to reap it, so the queue grows
    // until it is full and every packet behind it pays the whole depth: measured 103 packets per
    // hop at 9 Mb/s per slice and 257 at 12, which is the 464 ms and 958 ms URLLC figures, at
    // 0.00% loss. A buffer doing exactly what it was configured to do.
    //
    // Cobalt replaces it: a sojourn-TIME AQM (Target 5ms, Interval 100ms -- ns-3's defaults, which
    // are Nichols & Jacobson's published CoDel values, so cited rather than invented), so the
    // queue is bounded in milliseconds instead of packets.
    //
    // It cannot weaken the slicing: the Prio root has already sorted by slice ID, so a band's AQM
    // only ever sees that one slice's traffic. The no-slicing baseline puts a single FqCoDel on
    // the raw device, where it sees all 36 flows of all three slices and cannot tell them apart.
    //
    // FqCoDel was tried here FIRST and must not be retried: TbfQueueDisc needs a child with a
    // non-destructive DoPeek, because it reads the head packet's SIZE to decide whether it has
    // tokens (tbf-queue-disc.cc:191). FqCoDel and CoDel do not override DoPeek, so they inherit
    // the base one, which "peeks" by DEQUEUING into m_requeued -- and since Tbf is
    // SINGLE_CHILD_QUEUE_DISC its reported size IS the child's, so it reads 0 packets while still
    // holding one. That spins at 100% CPU forever: measured on the 3-node slice-qdisc-gate, 25 s
    // elapsed for 24 s of CPU and no output, where it normally finishes in seconds.
    // Peek-safe discs are Fifo, Cobalt, Red, PfifoFast, Prio and Mq; Cobalt is the only
    // sojourn-time AQM among them, hence this choice. The cost is named: Cobalt gives the latency
    // bound but NOT per-flow isolation, so head-of-line blocking between a slice's own 12 flows
    // remains, and the baseline keeps that one advantage.
    //
    // Identical on all three bands: rates may differ between slices, latency policy must not,
    // or a loss or delay comparison across slices reads the queue config instead of the design.
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        uint16_t tbf = tch.AddChildQueueDisc(h, cls[SLICE_BAND[s]], "ns3::TbfQueueDisc", "Burst",
                                             UintegerValue(MTU_BYTES));
        TrafficControlHelper::ClassIdList inner =
            tch.AddQueueDiscClasses(tbf, 1, "ns3::QueueDiscClass");
        // MaxSize must be given explicitly. TbfQueueDisc copies its own 1000p onto the child only
        // in the branch where it CREATES that child (tbf-queue-disc.cc:309); supplying one skips
        // that, so the child would otherwise keep its own default and silently change the depth
        // as well as the discipline. Pinned to the 1000p the Fifo had, so the only difference is
        // the discipline (rule 6). Cobalt should hold real occupancy far below this backstop.
        tch.AddChildQueueDisc(tbf, inner[0], g_sliceAqm, "MaxSize",
                              QueueSizeValue(QueueSize("1000p")));
    }
    QueueDiscContainer qdiscs = tch.Install(devices);

    for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
    {
        Ptr<NetDevice> dev = devices.Get(i);
        Ptr<NetDeviceQueueInterface> ndqi = dev->GetObject<NetDeviceQueueInterface>();
        Ptr<QueueDisc> root = qdiscs.Get(i);

        // TrafficControlLayer only wires a send callback on the discs in its wake list,
        // which for a WAKE_ROOT root is the root alone. A TbfQueueDisc re-schedules
        // QueueDisc::Run on ITSELF when it runs out of tokens (tbf-queue-disc.cc), so as a
        // child it would call an unset callback and abort with std::bad_function_call.
        // Wire the children by hand so a token-delayed packet still reaches the device.
        for (uint32_t b = 0; b < root->GetNQueueDiscClasses(); ++b)
        {
            Ptr<QueueDisc> child = root->GetQueueDiscClass(b)->GetQueueDisc();
            child->SetNetDeviceQueueInterface(ndqi);
            child->SetSendCallback([dev](Ptr<QueueDiscItem> item) {
                dev->Send(item->GetPacket(), item->GetAddress(), item->GetProtocol());
            });
        }
    }
    return qdiscs;
}

void
SetSliceAqm(const string& typeId)
{
    g_sliceAqm = typeId;
}

void
SetSliceLatencyTargets(const QueueDiscContainer& qdiscs, double targetSec)
{
    // Only a sojourn-time AQM has a Target; a Fifo child has no such attribute and SetAttribute
    // would abort rather than be ignored.
    if (g_sliceAqm != "ns3::CobaltQueueDisc")
    {
        return;
    }
    for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
    {
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            Ptr<QueueDisc> tbf =
                qdiscs.Get(i)->GetQueueDiscClass(SLICE_BAND[s])->GetQueueDisc();
            // The AQM is the Tbf's single child; see InstallSliceQdiscs for why it is Cobalt.
            if (tbf->GetNQueueDiscClasses() == 1)
            {
                tbf->GetQueueDiscClass(0)->GetQueueDisc()->SetAttribute(
                    "Target", TimeValue(Seconds(targetSec)));
            }
        }
    }
}

void
SetSliceRatesOn(const QueueDiscContainer& qdiscs,
                uint32_t index,
                const array<double, NUM_SLICES>& limitMbps)
{
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        // A zero share is a legitimate allocator output (an over-subscribed link leaves the
        // elastic slice nothing), but a TbfQueueDisc divides by its rate to work out when the
        // next token arrives, so rate 0 is SIGFPE. Floor it: "almost nothing" not "nothing".
        uint64_t bps = static_cast<uint64_t>(max(limitMbps[s], g_minLimitMbps) * 1e6);
        Ptr<QueueDisc> band = qdiscs.Get(index)->GetQueueDiscClass(SLICE_BAND[s])->GetQueueDisc();
        band->SetAttribute("Rate", DataRateValue(DataRate(bps)));
        if (g_lastLimitMbps.size() <= index)
        {
            g_lastLimitMbps.resize(index + 1, {});
        }
        g_lastLimitMbps[index][s] = bps / 1e6;
    }
}

void
SetMinLimitFromPeriod(double periodSec)
{
    if (periodSec > 0.0)
    {
        g_minLimitMbps = MTU_BYTES * 8.0 / periodSec / 1e6;
    }
}


void
SetSliceRates(const QueueDiscContainer& qdiscs, const array<double, NUM_SLICES>& limitMbps)
{
    for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
    {
        SetSliceRatesOn(qdiscs, i, limitMbps);
    }
}

array<uint64_t, NUM_SLICES>
SliceOfferedBytes(const QueueDiscContainer& qdiscs, uint32_t index)
{
    array<uint64_t, NUM_SLICES> bytes{};
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        bytes[s] = qdiscs.Get(index)
                       ->GetQueueDiscClass(SLICE_BAND[s])
                       ->GetQueueDisc()
                       ->GetStats()
                       .nTotalReceivedBytes;
    }
    return bytes;
}

array<uint64_t, NUM_SLICES>
SliceBacklogBytes(const QueueDiscContainer& qdiscs, uint32_t index)
{
    array<uint64_t, NUM_SLICES> bytes{};
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        // The Tbf stores nothing itself (tbf-queue-disc.cc:292), but AddQueueDiscClass wires the
        // child's Enqueue/Dequeue traces into the parent's counters, so the Tbf's own GetNBytes()
        // reports what its child is holding. Only GetMaxSize() delegates to the child; this does
        // not, and does not need to.
        bytes[s] = qdiscs.Get(index)
                       ->GetQueueDiscClass(SLICE_BAND[s])
                       ->GetQueueDisc()
                       ->GetNBytes();
    }
    return bytes;
}

void
PrintSliceQdiscStats(const QueueDiscContainer& qdiscs,
                     const array<uint64_t, NUM_SLICES>& srcPackets)
{
    static const array<const char*, NUM_SLICES> name = {"eMBB", "URLLC", "mMTC"};
    cout << "\n=== per-slice queue-disc totals (all nodes) ===\n";
    uint64_t ctrlRx = 0, ctrlDrop = 0;
    for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
    {
        QueueDisc::Stats st =
            qdiscs.Get(i)->GetQueueDiscClass(SLICE_BAND_CTRL)->GetQueueDisc()->GetStats();
        ctrlRx += st.nTotalReceivedPackets;
        ctrlDrop += st.nTotalDroppedPackets;
    }
    cout << "control: enq=" << ctrlRx << " drop=" << ctrlDrop << "\n";
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        uint64_t rx = 0, drop = 0;
        for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
        {
            QueueDisc::Stats st =
                qdiscs.Get(i)->GetQueueDiscClass(SLICE_BAND[s])->GetQueueDisc()->GetStats();
            rx += st.nTotalReceivedPackets;
            drop += st.nTotalDroppedPackets;
        }
        cout << name[s] << ": enq=" << rx << " drop=" << drop;
        // Per SOURCE packet, not per enqueue: enqueues tally once per hop, so dividing by them
        // hides our own policing behind the path length. hops/pkt makes that factor visible.
        if (srcPackets[s] > 0)
        {
            cout << "  policed=" << (100.0 * drop / srcPackets[s]) << "% of " << srcPackets[s]
                 << " src pkts  (hops/pkt " << (1.0 * rx / srcPackets[s]) << ")";
        }
        else
        {
            cout << " (" << (rx > 0 ? 100.0 * drop / rx : 0.0) << "% per enqueue; no source count)";
        }
        cout << "\n";
    }
}

void
PrintCapVsOffered(const QueueDiscContainer& qdiscs, double spanSec)
{
    // A cap only POLICES a node that is offered more than its own share, so a network-wide
    // "policed = 0%" cannot tell a generous cap from one that is never reached. The ratio can.
    // Printed per slice, over the nodes that carry the slice at all, then per node for URLLC,
    // where the two columns separate the two possible causes of its loss: offered above cap is a
    // rate problem, drops while under cap is arrival phasing against too few slots.
    static const array<const char*, NUM_SLICES> name = {"eMBB", "URLLC", "mMTC"};
    // The traffic window, not Simulator::Now(): the clock also holds the warm-up sweep, which
    // would divide the offered load by ~16 and make every node look idle.
    double span = (spanSec > 0.0) ? spanSec : Simulator::Now().GetSeconds();

    cout << "\n=== limit vs offered, per slice (nodes carrying the slice) ===\n";
    cout << "slice  nodes  mean limit  mean offered  mean off/limit  max off/limit  nodes over "
            "limit\n";
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        uint32_t nodes = 0, over = 0;
        double capSum = 0.0, offSum = 0.0, ratioSum = 0.0, ratioMax = 0.0;
        for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
        {
            QueueDisc::Stats st =
                qdiscs.Get(i)->GetQueueDiscClass(SLICE_BAND[s])->GetQueueDisc()->GetStats();
            if (st.nTotalReceivedPackets == 0)
            {
                continue;
            }
            double cap = (i < g_lastLimitMbps.size()) ? g_lastLimitMbps[i][s] : 0.0;
            double offered = (span > 0) ? (st.nTotalReceivedBytes * 8.0 / span / 1e6) : 0.0;
            double ratio = (cap > 0) ? offered / cap : 0.0;
            ++nodes;
            capSum += cap;
            offSum += offered;
            ratioSum += ratio;
            ratioMax = max(ratioMax, ratio);
            over += (ratio > 1.0) ? 1 : 0;
        }
        cout << name[s] << "  " << nodes << "  " << (nodes ? capSum / nodes : 0.0) << "  "
             << (nodes ? offSum / nodes : 0.0) << "  " << (nodes ? ratioSum / nodes : 0.0) << "  "
             << ratioMax << "  " << over << "\n";
    }

    cout << "\n=== URLLC per node (limit vs offered) ===\n";
    // "depth", not "K": since the depths were equalised this is the installed MaxSize, which is
    // ns-3's default unless --urllcQueuePkts overrode it, and no longer the derived K.
    cout << "node  limit(Mb/s)  offered(Mb/s)  enq  drop  depth(pkt)  held(pkt)\n";
    for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
    {
        Ptr<QueueDisc> band = qdiscs.Get(i)->GetQueueDiscClass(SLICE_BAND[1])->GetQueueDisc();
        QueueDisc::Stats st = band->GetStats();
        if (st.nTotalReceivedPackets == 0)
        {
            continue;
        }
        double cap = (i < g_lastLimitMbps.size()) ? g_lastLimitMbps[i][1] : -1.0;
        double offered = (span > 0) ? (st.nTotalReceivedBytes * 8.0 / span / 1e6) : 0.0;
        // K is the installed bucket size, held is what is in it now -- the old single "depth"
        // column printed K and so read 24 on every node whether the bucket was full or empty.
        cout << "  " << i << "   " << cap << "      " << offered << "      "
             << st.nTotalReceivedPackets << "  " << st.nTotalDroppedPackets << "  "
             << band->GetMaxSize().GetValue() << "  " << band->GetNPackets() << "\n";
    }
}
