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

// URLLC's queue depth is a PACKET COUNT derived from its loss target, not a delay budget.
//
// The old rule was `budget x rate`, i.e. the depth a delay budget pays for. It fails because at
// URLLC's small per-node rate (0.6-1.2 Mb/s measured) a 6.1 ms budget buys 1.7 packets, the MTU
// floor rounds it to 2, and a two-slot queue dropped 35% of URLLC's own packets while eMBB and
// mMTC (on ns-3's 1000p default) dropped none. Its deeper problem: the budget came from an M/M/1
// delay argument, and M/M/1 assumes an INFINITE buffer, so the derivation never covered the
// buffer it was sizing. See Alloc.md 4c.3b.
//
// A drop happens when more packets arrive at once than there are slots. That is counting, not
// timing, so the depth is counted in packets and the delay it causes is REPORTED rather than
// commanded. Both inputs are already in the design, so this adds no chosen number:
//
//   rho* = 1 - T0/T* = 0.61                    the utilization the headroom already targets
//   PLR <= 1e-5                                IEEE 802.11 RTA TIG, via Bankov et al.
//                                              (the same sentence that gives T* = 10 ms)
//
// For a finite buffer of K slots, loss falls by a factor of rho per slot: loss ~ (1-rho) rho^K.
// A packet crosses ~3.4 hops, so each hop may lose at most 1e-5/3.4 = 2.9e-6, which needs K = 24
// (K = 23 gives 4.5e-6 per hop = 1.5e-5 end to end and misses). 24 is the minimum that meets the
// cited target, not a round number.
//
// The formula assumes Poisson arrivals and ours are superposed constant-rate flows, which cluster
// worse: at K = 10 it predicted 0.95% end to end and the harness measured 3.3%, about 3x. So the
// formula picks the starting K and the sweep below settles it.
uint32_t g_urllcQueuePkts = 24;

// A TbfQueueDisc's Rate is set-only, so the installed cap cannot be read back off the disc; it
// is recorded here as it is written. All three slices, because "is the cap binding?" is a
// question about every slice, not only URLLC.
vector<array<double, NUM_SLICES>> g_lastCapMbps;

// eMBB and mMTC have no delay budget, so nothing derives a depth for them: they keep ns-3's
// default (1000p) by simply not being given one. "Unchanged from the simulator's default" is a
// stronger defence than any number chosen here, and it replaces the invented 200p.
//
// Measured both ways (alloc-gate baseline, 2026-10-05) before settling on the default: 1000p vs
// 200p moves eMBB's own delay (655 vs 219 ms) and NOTHING else -- C settles at 39.75 either way,
// URLLC holds 3.65 ms, mMTC 244.6 ms. The depth never reaches the controller, because the token
// bucket's output rate is the same whatever is queued behind it. So the cost of the default falls
// entirely on the one slice with no delay requirement, which is where it belongs.

// Smallest cap a token bucket can be given; see SetSliceRates.
const double MIN_SHARE_MBPS = 0.01;

} // namespace

QueueDiscContainer
InstallSliceQdiscs(const NetDeviceContainer& devices)
{
    TrafficControlHelper tch;
    uint16_t h = tch.SetRootQueueDisc("ns3::PrioQueueDisc", "Priomap", StringValue(PRIOMAP));
    tch.AddPacketFilter(h, "ns3::SliceBandFilter");
    TrafficControlHelper::ClassIdList cls = tch.AddQueueDiscClasses(h, 4, "ns3::QueueDiscClass");
    tch.AddChildQueueDisc(h, cls[SLICE_BAND_CTRL], "ns3::FifoQueueDisc");
    // URLLC: shallow, because a deep queue converts a burst into latency instead of a drop. The
    // depth is a placeholder here -- SetSliceRates sizes it from the delay budget and the rate.
    tch.AddChildQueueDisc(h, cls[SLICE_BAND[1]], "ns3::TbfQueueDisc", "Burst",
                          UintegerValue(MTU_BYTES), "MaxSize",
                          QueueSizeValue(QueueSize(PACKETS, g_urllcQueuePkts)));
    // eMBB and mMTC: MaxSize deliberately not given, so each keeps ns-3's 1000p default.
    tch.AddChildQueueDisc(h, cls[SLICE_BAND[0]], "ns3::TbfQueueDisc", "Burst",
                          UintegerValue(MTU_BYTES));
    tch.AddChildQueueDisc(h, cls[SLICE_BAND[2]], "ns3::TbfQueueDisc", "Burst",
                          UintegerValue(MTU_BYTES));
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
SetSliceRatesOn(const QueueDiscContainer& qdiscs,
                uint32_t index,
                const array<double, NUM_SLICES>& shareMbps)
{
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        // A zero share is a legitimate allocator output (an over-subscribed link leaves the
        // elastic slice nothing), but a TbfQueueDisc divides by its rate to work out when the
        // next token arrives, so rate 0 is SIGFPE. Floor it: "almost nothing" not "nothing".
        uint64_t bps = static_cast<uint64_t>(max(shareMbps[s], MIN_SHARE_MBPS) * 1e6);
        Ptr<QueueDisc> band = qdiscs.Get(index)->GetQueueDiscClass(SLICE_BAND[s])->GetQueueDisc();
        band->SetAttribute("Rate", DataRateValue(DataRate(bps)));
        if (g_lastCapMbps.size() <= index)
        {
            g_lastCapMbps.resize(index + 1, {});
        }
        g_lastCapMbps[index][s] = bps / 1e6;
        if (s == 1)
        {
        }

        // URLLC's depth follows its rate, so that the queue can never hold more than the delay
        // budget's worth of work. Two floors, both forced rather than chosen: one MTU, or a
        // full-size packet never fits; and whatever the queue is holding right now, because ns-3
        // aborts on a MaxSize below the current occupancy (queue.cc:201). The second floor is a
        // transient -- the next retune reaches the target once the backlog has drained.
    }
}

void
SetUrllcQueuePackets(uint32_t packets)
{
    g_urllcQueuePkts = max(1u, packets);
}

void
SetSliceRates(const QueueDiscContainer& qdiscs, const array<double, NUM_SLICES>& shareMbps)
{
    for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
    {
        SetSliceRatesOn(qdiscs, i, shareMbps);
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

    cout << "\n=== cap vs offered, per slice (nodes carrying the slice) ===\n";
    cout << "slice  nodes  mean cap  mean offered  mean off/cap  max off/cap  nodes over cap\n";
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
            double cap = (i < g_lastCapMbps.size()) ? g_lastCapMbps[i][s] : 0.0;
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

    cout << "\n=== URLLC per node (cap vs offered) ===\n";
    cout << "node  cap(Mb/s)  offered(Mb/s)  enq  drop  K(pkt)  held(pkt)\n";
    for (uint32_t i = 0; i < qdiscs.GetN(); ++i)
    {
        Ptr<QueueDisc> band = qdiscs.Get(i)->GetQueueDiscClass(SLICE_BAND[1])->GetQueueDisc();
        QueueDisc::Stats st = band->GetStats();
        if (st.nTotalReceivedPackets == 0)
        {
            continue;
        }
        double cap = (i < g_lastCapMbps.size()) ? g_lastCapMbps[i][1] : -1.0;
        double offered = (span > 0) ? (st.nTotalReceivedBytes * 8.0 / span / 1e6) : 0.0;
        // K is the installed bucket size, held is what is in it now -- the old single "depth"
        // column printed K and so read 24 on every node whether the bucket was full or empty.
        cout << "  " << i << "   " << cap << "      " << offered << "      "
             << st.nTotalReceivedPackets << "  " << st.nTotalDroppedPackets << "  "
             << band->GetMaxSize().GetValue() << "  " << band->GetNPackets() << "\n";
    }
}
