#include "loss-ledger.h"

#include <deque>
#include <iomanip>
#include <sstream>

#include "ns3/arp-cache.h"
#include "ns3/config.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/queue-disc.h"
#include "ns3/traffic-control-layer.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"

#include <array>
#include <iostream>

using namespace ns3;
using namespace std;

namespace {

array<uint64_t, 4> g_macDrops = {0, 0, 0, 0}; // indexed by WifiMacDropReason
uint64_t g_ipDrops = 0;
uint64_t g_arpDrops = 0;
uint64_t g_arpQueueDrops = 0;
array<uint64_t, 16> g_ipReason = {};

void
OnMacDrop(WifiMacDropReason reason, Ptr<const WifiMpdu>)
{
    size_t i = static_cast<size_t>(reason);
    if (i < g_macDrops.size())
    {
        ++g_macDrops[i];
    }
}

void
OnArpDrop(Ptr<const Packet>)
{
    ++g_arpDrops;
}

void
OnArpQueueDrop(Ptr<const Packet>)
{
    ++g_arpQueueDrops;
}

void
OnIpDrop(const Ipv4Header&, Ptr<const Packet>, Ipv4L3Protocol::DropReason reason, Ptr<Ipv4>, uint32_t)
{
    ++g_ipDrops;
    size_t i = static_cast<size_t>(reason);
    if (i < g_ipReason.size())
    {
        ++g_ipReason[i];
    }
}

} // namespace

void
InstallLossLedger(const MeshTopology& mesh)
{
    uint32_t macOk = 0, ipOk = 0;
    for (uint32_t i = 0; i < mesh.devices.GetN(); ++i)
    {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(mesh.devices.Get(i));
        if (dev && dev->GetMac() &&
            dev->GetMac()->TraceConnectWithoutContext("DroppedMpdu", MakeCallback(&OnMacDrop)))
        {
            ++macOk;
        }
    }
    for (uint32_t i = 0; i < mesh.nodes.GetN(); ++i)
    {
        Ptr<Ipv4L3Protocol> ip = mesh.nodes.Get(i)->GetObject<Ipv4L3Protocol>();
        if (ip && ip->TraceConnectWithoutContext("Drop", MakeCallback(&OnIpDrop)))
        {
            ++ipOk;
        }
    }
    // A silent failure here reads as "the medium lost nothing", which is the one answer that
    // must never be guessed.
    // ArpCache's Drop fires when a packet waiting on an ARP reply is pushed out: the pending
    // queue holds only 3 packets (arp-cache.cc) and entries go stale after 120 s, so a flow that
    // starts after a long quiet gap loses its opening burst with no other counter moving.
    Config::ConnectWithoutContext("/NodeList/*/$ns3::ArpL3Protocol/CacheList/*/Drop",
                                  MakeCallback(&OnArpDrop));
    // ArpL3Protocol has its OWN Drop, a different trace source from the cache's, and it is the
    // one that fires when the pending queue is already full (arp-l3-protocol.cc:320). Nothing
    // else records it: ipv4-interface.cc:295 drops a packet whose Lookup returns false with no
    // trace at all, so an unresolvable next hop is silent in every other counter. Without this
    // hook a flow can lose 100% of its packets and every figure still reads zero.
    Config::ConnectWithoutContext("/NodeList/*/$ns3::ArpL3Protocol/Drop",
                                  MakeCallback(&OnArpQueueDrop));
    cout << "loss ledger: MAC traces " << macOk << "/" << mesh.devices.GetN() << ", IP traces "
         << ipOk << "/" << mesh.nodes.GetN() << endl;
}


uint32_t
HeldPackets(const MeshTopology& mesh, uint32_t* inQdiscs, uint32_t* inRadios)
{
    uint32_t qdisc = 0, mac = 0;
    for (uint32_t i = 0; i < mesh.devices.GetN(); ++i)
    {
        Ptr<NetDevice> dev = mesh.devices.Get(i);
        Ptr<TrafficControlLayer> tc = mesh.nodes.Get(i)->GetObject<TrafficControlLayer>();
        if (tc)
        {
            // The ROOT's count alone is the total. ns-3 rolls a child's enqueue up the tree --
            // QueueDisc::AddQueueDiscClass wires the child's Enqueue trace to the parent's
            // PacketEnqueued, which increments the parent AND re-fires its own trace
            // (queue-disc.cc:626, 677-683). Summing the tree counted every packet once per level:
            // root Prio + Tbf + its internal Fifo = 3x, which is why a 1000-slot bucket reported
            // 3000 packets held.
            Ptr<QueueDisc> root = tc->GetRootQueueDiscOnDevice(dev);
            if (root)
            {
                qdisc += root->GetNPackets();
            }
        }
        Ptr<WifiNetDevice> wifi = DynamicCast<WifiNetDevice>(dev);
        if (wifi && wifi->GetMac())
        {
            Ptr<WifiMacQueue> q = wifi->GetMac()->GetTxopQueue(AC_BE);
            if (q)
            {
                mac += q->GetNPackets();
            }
        }
    }
    if (inQdiscs)
    {
        *inQdiscs = qdisc;
    }
    if (inRadios)
    {
        *inRadios = mac;
    }
    return qdisc + mac;
}

void
PrintHeldPackets(const MeshTopology& mesh)
{
    uint32_t qdisc = 0, mac = 0;
    uint32_t total = HeldPackets(mesh, &qdisc, &mac);
    cout << "  still held at end: " << qdisc << " in queue discs, " << mac << " in radio queues, "
         << total << " total\n";
}

namespace {

double g_drainStart = 0.0;
double g_drainTook = -1.0;
uint32_t g_lastHeld = 0;     // held count at the previous poll
double g_flatSince = -1.0;   // when the held count last CHANGED
bool g_drainStalled = false; // held count stopped falling before reaching zero

// Poll rather than wait a fixed time: the drain is a CONDITION (the queues are empty), not a
// number. RFC 2544 asks for a drain -- "wait for two seconds for any residual frames to be
// received" (section 23, step d) -- but as a STOP time two seconds is a figure for a switch whose
// buffer holds milliseconds, and a token bucket here holds seconds: at 4x load a 2 s drain left
// 18.7% of all packets inside the queue discs. So keep the requirement and drop the constant: stop
// as soon as nothing is held, and report how long it took, which is the settling time the Winter
// Simulation Conference 2003 checklist says must be stated.

// How long the held count may sit UNCHANGED before the queues are declared not-draining -- a
// separate question from the stop above. A live backlog shrinks; frames awaiting WifiMacQueue's
// lazy reap never move again, because nothing enqueues after the sources stop.
//
// 2 s is RFC 2544 section 23 step (d) again, and using it here is a REUSE, not a derivation: the
// RFC's two seconds answers "how long to wait for stragglers to ARRIVE", this one answers "how long
// of NO MOVEMENT means stuck". Same duration, different question, so the jump is named rather than
// dressed up -- and the earlier justification in this comment ("~40 polls, far longer than any gap
// a draining queue shows") was a guess, since nobody had measured the gaps a draining queue shows.
//
// Settled 2026-10-13 by showing the verdict does not depend on it. `--stallSec` exposes the value;
// 1 / 2 / 5 s were run on one gaSeed/rngRun in two regimes (10x10, --sliceQdisc):
//   eMBB 100 Mb/s overload - drained to zero in 27.35 s in all three, so this branch is never
//                            reached. 1647 frames expired yet none were left: traffic was still
//                            flowing, so enqueues kept triggering the reap.
//   Me=Mu=Mc=20 at 4/4/4   - the regime that actually wedges. All three stalled at 60 packets,
//                            every one in a radio queue and none in a queue disc, all three VALID,
//                            and every slice figure identical to the last digit (eMBB 94.864 ms /
//                            26.1296%, URLLC 8.18428 ms, mMTC 13.1984 ms). Only the drain time
//                            moved, by exactly the threshold: 4.2 / 5.2 / 8.2 s.
// So the constant sets when to stop looking and nothing else. It is also the LOWER load that
// wedges, not the higher one -- corpses need frames to expire at the tail of the traffic with no
// enqueue after, which an overload keeps sweeping away.
double g_stallSec = 2.0;
// Whether the drain sweeps the MAC queues. Exposed so the sweep can be REMOVED and the run
// re-measured: reaching into ns-3's MAC is a simulator-implementation argument, not a networking
// one, so it is the least defensible line in the harness and has to earn its place (user,
// 2026-10-15). It is NOT believed to be caused by the old Fifo child -- the wedge is device
// flow-control plus WifiMacQueue's enqueue-triggered reap, and Cobalt drops on DEQUEUE, which
// cannot happen while the device is stopped. Cobalt may only make it rarer by shedding load
// earlier.
bool g_drainReap = true;

// --- Settle watcher -------------------------------------------------------------------------
// The measurement window must open in steady state, not in the initial transient. "Steady" is
// read off the same observable the drain already uses -- the held-packet count -- so no new
// constant appears: queues at equilibrium means arrivals equal departures, which means occupancy
// stops RISING. `stallSec` (RFC 2544 section 23 step d, already cited for the drain) is how long
// "stopped" has to hold.
// Occupancy is REPORTED, not judged. Two attempts at a boolean "have the queues stopped growing"
// both failed, and the second failure shows the question is ill-posed rather than mis-coded:
//   sample vs previous sample  - occupancy jitters constantly, so any increase reset the clock and
//                                the condition never held; it called a run "still growing" that
//                                then drained in 0.35 s (2026-10-15).
//   sample vs one stallSec ago - same defect at a longer timescale. At 5 Mb/s per slice, where the
//                                drain is 0.1 s and almost nothing is held, held=3 against held=2
//                                two seconds earlier still reads as growth.
// A strict inequality on a noisy signal fires about half the time in steady state, and the only
// way to make it a clean yes/no is a tolerance threshold - exactly the invented constant this
// project keeps deleting. So print the occupancy and let it be read. The formal steady-state
// evidence is agreement between NESTED measurement windows (CLAUDE.md: --measureSec), which tests
// the quantity actually reported instead of a proxy for it.
deque<pair<double, uint32_t>> g_settleHist; // (time, held) over the whole watch
bool g_settleWatching = false;
double g_settleStallSec = 2.0;

void
SettlePoll(const MeshTopology& mesh, double pollSec)
{
    g_settleHist.emplace_back(Simulator::Now().GetSeconds(), HeldPackets(mesh, nullptr, nullptr));
    if (g_settleWatching)
    {
        Simulator::Schedule(Seconds(pollSec), &SettlePoll, mesh, pollSec);
    }
}


void
DrainPoll(const MeshTopology& mesh, double pollSec, double capSec)
{
    double now = Simulator::Now().GetSeconds();
    if (g_drainReap)
    {
        ReapExpired(mesh);
    }
    uint32_t held = HeldPackets(mesh, nullptr, nullptr);
    if (held == 0)
    {
        g_drainTook = now - g_drainStart;
        Simulator::Stop(Seconds(0));
        return;
    }
    // Track MOVEMENT, not the count. "Still held" alone cannot tell a live backlog from frames
    // already dropped and merely unreaped, and those need opposite verdicts.
    if (held != g_lastHeld || g_flatSince < 0.0)
    {
        g_lastHeld = held;
        g_flatSince = now;
    }
    else if (now - g_flatSince >= g_stallSec)
    {
        g_drainStalled = true;
        g_drainTook = now - g_drainStart;
        Simulator::Stop(Seconds(0)); // not going to drain; PrintDrain decides whether that matters
        return;
    }
    if (now - g_drainStart >= capSec)
    {
        Simulator::Stop(Seconds(0)); // still falling at the cap -> genuinely truncated
        return;
    }
    Simulator::Schedule(Seconds(pollSec), &DrainPoll, mesh, pollSec, capSec);
}

} // namespace

void
ScheduleSettleWatch(const MeshTopology& mesh, double startSec, double pollSec, double stallSec)
{
    g_settleStallSec = max(stallSec, pollSec);
    g_settleHist.clear();
    g_settleWatching = true;
    Simulator::Schedule(Seconds(startSec), &SettlePoll, mesh, pollSec);
}

string
OccupancyTrend(double stallSec)
{
    g_settleWatching = false; // the window is opening; stop polling
    if (g_settleHist.size() < 2)
    {
        return "not measured";
    }
    const double now = g_settleHist.back().first;
    double recentSum = 0, allSum = 0;
    uint32_t recentN = 0, peak = 0;
    for (const auto& [t, h] : g_settleHist)
    {
        allSum += h;
        peak = max(peak, h);
        if (now - t <= stallSec)
        {
            recentSum += h;
            ++recentN;
        }
    }
    const double recent = recentN ? recentSum / recentN : 0.0;
    const double overall = allSum / g_settleHist.size();
    ostringstream o;
    o << fixed << setprecision(1) << "held pkts: last " << stallSec << "s mean " << recent
      << ", whole-traffic mean " << overall << ", peak " << peak
      << " (recent/overall " << (overall > 0 ? recent / overall : 0.0) << ")";
    return o.str();
}

// WifiMacQueue sweeps its own dead frames only on Enqueue / Flush / a head-of-queue peek, and a FULL
// queue stops the device's tx queue, which flow-controls the queue disc behind it -- so no enqueue
// can happen, no sweep happens, and the queue stays full. One node wedged exactly that way at
// 13 Mb/s: MAC 500 of 500, its mMTC bucket 1000 of 1000, every MAC counter reading zero.
//
// Holding back at L2.5 to keep a slot free does NOT fix it: the sweep is TRIGGERED by an enqueue,
// so declining to enqueue declines the sweep too, and the stall just moves one slot earlier. Only
// calling the sweep directly works, because it needs no enqueue.
//
// Called from the DRAIN only. A 500 ms timer through the traffic window was built and measured on
// 2026-10-14 and removed as inert: at 24 Mb/s per slice, the heaviest load tested, it fired all 16
// times and swept ZERO frames. While traffic flows the enqueue-driven sweep keeps up on its own;
// the wedge forms at the TAIL, once the sources stop and no enqueue can ever come again.
//
// These frames are already dead under dot11MaxTransmitMSDULifetime -- ns-3's lazy reap is the
// deviation from its own MaxDelay semantics, not this. Sweeping on a timer makes it conform.
void
ReapExpired(const MeshTopology& mesh)
{
    for (uint32_t i = 0; i < mesh.devices.GetN(); ++i)
    {
        Ptr<WifiNetDevice> w = DynamicCast<WifiNetDevice>(mesh.devices.Get(i));
        if (w && w->GetMac())
        {
            Ptr<WifiMacQueue> q = w->GetMac()->GetTxopQueue(AC_BE);
            if (q)
            {
                q->WipeAllExpiredMpdus();
            }
        }
    }
}

void
ScheduleDrain(const MeshTopology& mesh,
              double startSec,
              double pollSec,
              double capSec,
              double stallSec,
              bool reap)
{
    g_drainReap = reap;
    g_stallSec = max(stallSec, pollSec); // below one poll it would fire on the first flat reading
    g_drainStart = startSec;
    g_drainTook = -1.0;
    g_lastHeld = 0;
    g_flatSince = -1.0;
    g_drainStalled = false;
    Simulator::Schedule(Seconds(startSec), &DrainPoll, mesh, pollSec, capSec);
}

// A run is invalid when its figures are TRUNCATED, which is not the same as "something is still
// held". Two cases produce a non-zero count and they need opposite verdicts:
//
//   live backlog   - packets that would still be delivered if the run went on. Their delays are
//                    missing from the average, so the run is truncated. INVALID.
//   unreaped frames - packets already dropped for exceeding the 500 ms MSDU lifetime and counted
//                    in the ledger, sitting in WifiMacQueue only because it reaps lazily, on the
//                    next enqueue, which never comes once the sources stop. Their loss is already
//                    recorded and they will never be delivered, so nothing is missing. VALID.
//
// The discriminator is movement plus cause plus LAYER: a live backlog keeps shrinking, unreaped
// frames require a non-zero lifetime-expiry count to exist at all, and only WifiMacQueue can hold a
// corpse. A queue disc has no packet lifetime and nothing reaps it, so whatever sits in a token
// bucket is alive and would still be delivered -- the corpse verdict therefore requires the queue
// discs to be EMPTY, not merely for the MAC to have expired something somewhere else in the mesh.
// Judging on the count alone discarded the corpse case, and since stalling tracks overload that
// silently deleted the WORST runs from both arms - exactly the direction that flatters a result.
bool
PrintDrain(const MeshTopology& mesh, double capSec)
{
    uint32_t qdisc = 0, radios = 0;
    uint32_t held = HeldPackets(mesh, &qdisc, &radios);
    if (held == 0)
    {
        cout << "  drain: queues empty after " << g_drainTook << " s (condition, not a constant)\n";
        return true;
    }
    const uint64_t expired = g_macDrops[1]; // WIFI_MAC_DROP_EXPIRED_LIFETIME
    if (g_drainStalled && expired > 0 && qdisc == 0)
    {
        cout << "  drain: stopped after " << g_drainTook << " s with " << radios
             << " packet(s) held in radio queues only, unchanged for " << g_stallSec << " s; "
             << expired
             << " frame(s) already dropped on the 500 ms MSDU lifetime, so these are unreaped"
             << " corpses (WifiMacQueue reaps on the next enqueue, which never comes) -- their"
             << " loss is already counted and the run is VALID\n";
        return true;
    }
    if (g_drainStalled && qdisc > 0)
    {
        cout << "  drain: INVALID -- " << qdisc << " packet(s) held in queue discs (plus " << radios
             << " in radio queues), unchanged for " << g_stallSec
             << " s. A queue disc has no packet lifetime and nothing reaps it, so these are alive"
             << " and undelivered however many frames the MAC expired (" << expired
             << "); every delay and loss figure in this run is truncated\n";
        return false;
    }
    if (g_drainStalled)
    {
        cout << "  drain: INVALID -- " << held << " packet(s) held and unchanged for " << g_stallSec
             << " s, with no MAC lifetime expiry to explain them; something is stuck that was never"
             << " counted as lost\n";
        return false;
    }
    cout << "  drain: INVALID -- " << held << " packet(s) still held and still draining at the "
         << capSec << " s cap; every delay and loss figure in this run is truncated\n";
    return false;
}

void
PrintLossLedger()
{
    static const array<const char*, 4> macName = {"queue full (failed enqueue)",
                                                  "lifetime expired (500 ms MSDU)",
                                                  "retry limit reached",
                                                  "stale QoS packet"};
    cout << "\n=== where the medium lost frames (all nodes, all slices) ===\n";
    uint64_t total = 0;
    for (size_t i = 0; i < g_macDrops.size(); ++i)
    {
        cout << "  MAC " << macName[i] << ": " << g_macDrops[i] << "\n";
        total += g_macDrops[i];
    }
    cout << "  ARP cache drops (retries exhausted): " << g_arpDrops << "\n";
    cout << "  ARP pending-queue full (unresolvable next hop): " << g_arpQueueDrops << "\n";
    cout << "  IP drops (route/other): " << g_ipDrops << "\n";
    for (size_t i = 0; i < g_ipReason.size(); ++i)
    {
        if (g_ipReason[i])
        {
            cout << "    ip reason " << i << ": " << g_ipReason[i] << "\n";
        }
    }
    cout << "  MAC total: " << total << "\n";
}
