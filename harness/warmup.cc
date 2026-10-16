#include "warmup.h"

#include "ns3/inet-socket-address.h"
#include "ns3/mobility-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/queue-disc.h"
#include "ns3/simulator.h"
#include "ns3/traffic-control-layer.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-net-device.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>

using namespace ns3;
using namespace std;

static uint32_t
CountQueueDisc(Ptr<QueueDisc> d)
{
    uint32_t n = d->GetNPackets();
    for (uint32_t i = 0; i < d->GetNQueueDiscClasses(); ++i)
    {
        n += CountQueueDisc(d->GetQueueDiscClass(i)->GetQueueDisc());
    }
    return n;
}

// Everything one node still holds for transmission: the queue disc tree plus the radio's own
// AC_BE queue. Zero means the probe it just sent has fully left, which is the condition the
// next edge waits on -- a measured state, not a guessed interval.
static uint32_t
PendingPackets(const MeshTopology& mesh, uint32_t node)
{
    uint32_t n = 0;
    Ptr<NetDevice> dev = mesh.devices.Get(node);
    Ptr<TrafficControlLayer> tc = mesh.nodes.Get(node)->GetObject<TrafficControlLayer>();
    if (tc)
    {
        Ptr<QueueDisc> root = tc->GetRootQueueDiscOnDevice(dev);
        if (root)
        {
            n += CountQueueDisc(root);
        }
    }
    Ptr<WifiNetDevice> wifi = DynamicCast<WifiNetDevice>(dev);
    if (wifi && wifi->GetMac())
    {
        Ptr<WifiMacQueue> q = wifi->GetMac()->GetTxopQueue(AC_BE);
        if (q)
        {
            n += q->GetNPackets();
        }
    }
    return n;
}

// Drop everything the sender still holds, so the next link starts from a known state. RFC 2544
// asks for the device under test to be returned to a known state between trials; on a bench that
// is a reset, not a wait. Without it the leftovers of a saturating probe are packets whose
// lifetime has already expired -- never deliverable, and only reaped lazily.
static void
FlushSender(const MeshTopology& mesh, uint32_t node)
{
    Ptr<NetDevice> dev = mesh.devices.Get(node);
    Ptr<TrafficControlLayer> tc = mesh.nodes.Get(node)->GetObject<TrafficControlLayer>();
    if (tc)
    {
        Ptr<QueueDisc> root = tc->GetRootQueueDiscOnDevice(dev);
        // Dequeue rather than flushing the internal queues, so the disc's own counters follow.
        while (root && root->Dequeue())
        {
        }
    }
    Ptr<WifiNetDevice> wifi = DynamicCast<WifiNetDevice>(dev);
    if (wifi && wifi->GetMac())
    {
        Ptr<WifiMacQueue> q = wifi->GetMac()->GetTxopQueue(AC_BE);
        if (q)
        {
            q->Flush();
        }
    }
}

// Read the R sink's byte counter at a fixed instant. Two of these bracket the measured
// interval, so what lands in the link table is bytes-per-second over an interval we chose,
// not over however long the arrivals happened to straggle.
static void
SnapRWindowStart(WarmupContext* ctx, size_t edgeIdx)
{
    if (ctx->rwin[edgeIdx].sink)
    {
        ctx->rwin[edgeIdx].startRx = ctx->rwin[edgeIdx].sink->GetTotalRx();
    }
}

static void
SnapRWindowEnd(WarmupContext* ctx, size_t edgeIdx, double dur)
{
    if (ctx->rwin[edgeIdx].sink)
    {
        ctx->rwin[edgeIdx].endRx = ctx->rwin[edgeIdx].sink->GetTotalRx();
        ctx->rwin[edgeIdx].dur = dur;
    }
}

// Schedule one edge's isolated sweep at sim time `t`: L (empty -> base_delay), then R (saturating
// -> capacity). One window each, a -> b, on fresh ports. Advances s->nextPort; returns the sim
// time it ends.
//
// There used to be an unmeasured low-rate PRIME window before L, doing two jobs that are now both
// done elsewhere: resolving ARP, which `PerfectArp` does at setup before any warm-up runs, and
// letting the queue settle, which `FlushSender` does between edges. Removed 2026-10-14 along with
// its `primeSec`/`primeRate` knobs. It was ~18 packets against the R window's ~10,700, so this is
// tidiness, not speed - warm-up wall time is the R probe and nothing else.
static double
ScheduleEdgeWindows(WarmupSession* s, size_t edgeIdx, double t)
{

    const string lRate = "1Mb/s";    // low -> link stays ~empty -> base_delay
    const string rRate = s->satRate; // above link ceiling -> saturates -> R
    const double guard = s->guardSec; // drain queue + channel between windows

    uint32_t a = s->edges[edgeIdx].first;
    uint32_t b = s->edges[edgeIdx].second;
    // Apps installed mid-sim initialize at delay 0, and DoInitialize treats the
    // start time as a delay from *now*. So schedule relative to now, not absolute,
    // or a retry installed at t=now would fire at now+now and miss the sim window.
    double now = Simulator::Now().GetSeconds();
    struct SubWindow { string rate; double dur; int phase; };
    vector<SubWindow> subs;
    subs.push_back({lRate, s->windowSec, 0});
    subs.push_back({rRate, s->windowSec, 1});
    for (const SubWindow& sw : subs)
    {
        uint16_t port = s->nextPort++;
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(s->interfaces.GetAddress(b), port));
        onoff.SetConstantRate(DataRate(sw.rate), s->pktSize);
        ApplicationContainer src = onoff.Install(s->mesh.nodes.Get(a));
        src.Start(Seconds(t - now));
        src.Stop(Seconds(t + sw.dur - now));

        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer rcv = sink.Install(s->mesh.nodes.Get(b));
        rcv.Start(Seconds(t - now));
        // The R sink is never stopped: the probe's drain is part of what the link delivered, and
        // a stop time would be another guess at how long that takes.
        if (sw.phase != 1)
        {
            rcv.Stop(Seconds(t + sw.dur + guard - now));
        }

        if (sw.phase == 0)
        {
            s->ctx.portToProbe[port] = {edgeIdx, sw.phase};
            s->lastSinks[edgeIdx].first = DynamicCast<PacketSink>(rcv.Get(0));
        }
        else if (sw.phase == 1)
        {
            s->ctx.portToProbe[port] = {edgeIdx, sw.phase};
            s->lastSinks[edgeIdx].second = DynamicCast<PacketSink>(rcv.Get(0));
            s->ctx.rwin[edgeIdx].sink = s->lastSinks[edgeIdx].second;
            // Skip the head of the window: the first packets meet an empty MAC queue, so
            // they leave faster than the link's steady rate and would read high.
            double skip = min(0.1, sw.dur / 5.0);
            Simulator::Schedule(Seconds(t + skip - now), &SnapRWindowStart, &s->ctx, edgeIdx);
            Simulator::Schedule(Seconds(t + sw.dur - now),
                                &SnapRWindowEnd,
                                &s->ctx,
                                edgeIdx,
                                sw.dur - skip);
        }
        t += sw.dur + guard;
    }
    return t;
}

static double
Distance(const MeshTopology& mesh, uint32_t u, uint32_t v)
{
    Vector pu = mesh.nodes.Get(u)->GetObject<MobilityModel>()->GetPosition();
    Vector pv = mesh.nodes.Get(v)->GetObject<MobilityModel>()->GetPosition();
    return CalculateDistance(pu, pv);
}

vector<Edge>
SampleEdgesByPosition(const MeshTopology& mesh, const vector<Edge>& all, uint32_t want)
{
    // Rank edges by how far the midpoint sits from the field centre, then take `want` evenly
    // spaced ranks. That spans interior -> edge -> corner without needing the grid width, and it
    // is deterministic, so the same topology always yields the same sample. Spread is the point:
    // a sample clustered in the interior could not DETECT a position effect, only assume none.
    const uint32_t n = mesh.nodes.GetN();
    Vector centre(0.0, 0.0, 0.0);
    for (uint32_t i = 0; i < n; ++i)
    {
        Vector p = mesh.nodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        centre.x += p.x / n;
        centre.y += p.y / n;
    }

    vector<pair<double, size_t>> ranked;
    ranked.reserve(all.size());
    for (size_t e = 0; e < all.size(); ++e)
    {
        Vector a = mesh.nodes.Get(all[e].first)->GetObject<MobilityModel>()->GetPosition();
        Vector b = mesh.nodes.Get(all[e].second)->GetObject<MobilityModel>()->GetPosition();
        Vector mid((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, 0.0);
        ranked.emplace_back(CalculateDistance(mid, centre), e);
    }
    sort(ranked.begin(), ranked.end());

    vector<Edge> picked;
    for (uint32_t k = 0; k < want; ++k)
    {
        // Evenly spaced ranks, both ends included; want == 1 takes the most interior edge.
        size_t r = (want == 1) ? 0 : (k * (ranked.size() - 1)) / (want - 1);
        picked.push_back(all[ranked[r].second]);
    }
    cout << "edge sample: " << picked.size() << " of " << all.size()
         << " edges, spread by distance from field centre" << endl;
    return picked;
}

vector<Edge>
BuildEdgeListGeometric(const MeshTopology& mesh, double maxDist)
{
    uint32_t n = mesh.nodes.GetN();
    // Auto: min pairwise distance = grid spacing; *1.3 keeps orthogonal neighbours
    // (1.0 spacing), drops diagonals (1.414 spacing).
    if (maxDist <= 0.0)
    {
        double minD = numeric_limits<double>::max();
        for (uint32_t i = 0; i < n; ++i)
        {
            for (uint32_t j = i + 1; j < n; ++j)
            {
                minD = min(minD, Distance(mesh, i, j));
            }
        }
        maxDist = minD * 1.3;
    }
    vector<Edge> edges;
    for (uint32_t i = 0; i < n; ++i)
    {
        for (uint32_t j = i + 1; j < n; ++j)
        {
            if (Distance(mesh, i, j) <= maxDist)
            {
                edges.emplace_back(i, j);
            }
        }
    }
    return edges;
}

static void StartNextEdge(WarmupSession* s);

// Hold the sweep until the sender it just used has nothing left to transmit. This is what makes
// the measurement honest without any tuned constant: a probe above the link ceiling always
// leaves a backlog, and that backlog drains at the LINK rate only while nothing else is on air.
// Start the next edge too early and the drain crawls instead, which is what read a healthy
// 82.9 Mb/s link as 11.8-35. Nothing here is a guess at how long draining takes; the queues are
// asked.
static void
WaitDrain(WarmupSession* s, uint32_t sender)
{
    uint32_t n = PendingPackets(s->mesh, sender);
    double now = Simulator::Now().GetSeconds();
    if (n != s->drainLast)
    {
        s->drainLast = n;
        s->drainLastChange = now;
    }
    // Empty is the normal exit. A count that has stopped moving is the other one: a queue can
    // hold a packet that will never be sent (an expired lifetime is only reaped on the next
    // dequeue), and waiting on it would stall the sweep forever. Either way nothing is on air.
    if (n > 0 && now - s->drainLastChange < s->drainStallSec)
    {
        Simulator::Schedule(Seconds(s->drainPollSec), &WaitDrain, s, sender);
        return;
    }
    if (n > 0 && s->traceR)
    {
        cout << "  drain node " << sender << " stalled with " << n << " packet(s) held" << endl;
    }
    if (s->flushSender)
    {
        FlushSender(s->mesh, sender);
    }
    s->drainLast = UINT32_MAX;
    StartNextEdge(s);
}

static void
StartNextEdge(WarmupSession* s)
{
    if (s->cursor >= s->pending.size())
    {
        Simulator::ScheduleNow(&WarmupRetryPass, s);
        return;
    }
    size_t e = s->pending[s->cursor++];
    double now = Simulator::Now().GetSeconds();
    double end = ScheduleEdgeWindows(s, e, now);
    Simulator::Schedule(Seconds(end - now), &WaitDrain, s, s->edges[e].first);
}

void
BeginWarmup(WarmupSession* s)
{
    s->edges = BuildEdgeListGeometric(s->mesh);
    if (s->sampleEdges > 0 && s->sampleEdges < s->edges.size())
    {
        s->edges = SampleEdgesByPosition(s->mesh, s->edges, s->sampleEdges);
    }
    s->ctx.edges = s->edges;
    s->ctx.traceR = s->traceR;
    s->ctx.pktSize = s->pktSize;
    s->ctx.rwin.assign(s->edges.size(), WarmupContext::RWindow{});
    cout << "edge list (" << s->edges.size() << " edges), serial sweep, drain-gated" << endl;

    // One edge at a time, and the next one does not start until the last sender's queues are
    // empty. Edges used to be scheduled back to back at fixed times, which overlapped each
    // probe's drain with the following measurement. (Spatial-reuse batching was tried and
    // dropped -- no real speedup on the mesh, and on wide-channel ax it corrupts the data.)
    s->pending.clear();
    for (size_t e = 0; e < s->edges.size(); ++e)
    {
        s->pending.push_back(e);
    }
    s->cursor = 0;
    s->round = 1;

    // Safety bound only: the sweep now ends when the queues say so, and WarmupRetryPass sets the
    // real stop. 10 s per edge per round is far above anything measured.
    Simulator::Stop(Seconds(s->edges.size() * 10.0 * (s->maxRounds + 1) + 2.0));
    StartNextEdge(s);
    cout << "warm-up round 1: " << s->edges.size() << " edges" << endl;
}

void
WarmupRetryPass(WarmupSession* s)
{
    // Detect failures from each probe's own sink counter, not FlowMonitor: reading
    // FlowMonitor mid-simulation stops it recording flows installed afterwards.
    // The R window intentionally overloads the link, so loss is expected — judge
    // on *delivered* bytes. The median R delivery is the healthy-link value; a
    // link far below it (a collision knocked out its on-air deliveries) or with an
    // empty window hit a timing miss worth re-measuring at a new time.
    vector<uint64_t> rxR;
    for (size_t e = 0; e < s->edges.size(); ++e)
    {
        auto it = s->lastSinks.find(e);
        if (it != s->lastSinks.end() && it->second.second && it->second.second->GetTotalRx() > 0)
        {
            rxR.push_back(it->second.second->GetTotalRx());
        }
    }
    uint64_t medianR = 0;
    if (!rxR.empty())
    {
        sort(rxR.begin(), rxR.end());
        medianR = rxR[rxR.size() / 2];
    }

    vector<size_t> bad;
    for (size_t e = 0; e < s->edges.size(); ++e)
    {
        auto it = s->lastSinks.find(e);
        if (it == s->lastSinks.end())
        {
            continue;
        }
        Ptr<PacketSink> lSink = it->second.first;
        Ptr<PacketSink> rSink = it->second.second;
        // Median-relative (not absolute) so it stays valid after the propagation
        // swap, when links legitimately differ — 1/4 of median only trips on gross
        // collapse, not honest variation.
        if (!lSink || !rSink || lSink->GetTotalRx() == 0 || rSink->GetTotalRx() == 0 ||
            (medianR > 0 && rSink->GetTotalRx() * 4 < medianR))
        {
            bad.push_back(e);
        }
    }

    if (bad.empty() || s->round >= s->maxRounds)
    {
        if (!bad.empty())
        {
            cout << "warm-up: " << bad.size() << " edge(s) still failing after "
                 << s->maxRounds << " rounds:";
            for (size_t e : bad)
            {
                cout << " " << s->edges[e].first << "-" << s->edges[e].second;
            }
            cout << endl;
        }
        cout << "warm-up: converged after round " << s->round << endl;
        Simulator::Stop(Seconds(0.5));
        return;
    }

    cout << "warm-up round " << (s->round + 1) << ": retrying " << bad.size()
         << " edge(s):";
    for (size_t e : bad)
    {
        cout << " " << s->edges[e].first << "-" << s->edges[e].second;
    }
    cout << endl;

    // Re-measure just the failed edges, one at a time, drain-gated like the first round.
    s->pending = bad;
    s->cursor = 0;
    ++s->round;
    StartNextEdge(s);
}

vector<LinkMeasurement>
ExtractLinkTable(const WarmupContext& ctx,
                 Ptr<Ipv4FlowClassifier> classifier,
                 Ptr<FlowMonitor> monitor)
{
    vector<LinkMeasurement> table;
    table.reserve(ctx.edges.size());
    for (const Edge& e : ctx.edges)
    {
        table.push_back({e.first, e.second});
    }

    for (const auto& [id, s] : monitor->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple ft = classifier->FindFlow(id);
        auto it = ctx.portToProbe.find(ft.destinationPort);
        if (it == ctx.portToProbe.end() || s.rxPackets == 0)
        {
            continue;
        }
        size_t e = it->second.first;
        int phase = it->second.second;
        if (phase == 0)
        {
            table[e].baseDelayMs = (s.delaySum.GetSeconds() / s.rxPackets) * 1000.0;
        }
        else
        {
            double dur = (s.timeLastRxPacket - s.timeFirstRxPacket).GetSeconds();
            double spanBps = (dur > 0) ? (s.rxBytes * 8.0 / dur) : 0.0;
            table[e].capacityBps = spanBps;
            // Cross-check only: bytes over a fixed sub-window. It should agree with the span
            // figure now that the sender is reset between links, and a disagreement means a
            // leftover from the previous link got counted.
            double winBps = 0.0;
            if (e < ctx.rwin.size() && ctx.rwin[e].dur > 0.0 &&
                ctx.rwin[e].endRx >= ctx.rwin[e].startRx)
            {
                double hdr = (ctx.pktSize + 28.0) / ctx.pktSize; // payload -> IP-layer bytes
                winBps = (ctx.rwin[e].endRx - ctx.rwin[e].startRx) * hdr * 8.0 / ctx.rwin[e].dur;
            }
            // Capacity = bytes/span, so a wrong answer is either too few bytes or too long a
            // span. Printing both separates them; without it the only visible symptom is a
            // number that looks plausible. (A 300Mb/s probe used to read ~7x low here.)
            if (ctx.traceR)
            {
                cout << "  R " << table[e].a << "-" << table[e].b << " tx=" << s.txPackets
                     << " rx=" << s.rxPackets << " loss="
                     << (s.txPackets ? 100.0 * (s.txPackets - s.rxPackets) / s.txPackets : 0.0)
                     << "%  span=" << dur << "s  -> " << (table[e].capacityBps / 1e6)
                     << " Mbps  (window " << (winBps / 1e6) << ")" << endl;
            }
        }
    }
    return table;
}

void
MedianCorrectOutliers(vector<LinkMeasurement>& table)
{
    vector<double> delays;
    vector<double> caps;
    for (const LinkMeasurement& m : table)
    {
        if (m.baseDelayMs > 0)
        {
            delays.push_back(m.baseDelayMs);
        }
        if (m.capacityBps > 0)
        {
            caps.push_back(m.capacityBps);
        }
    }
    if (delays.empty() || caps.empty())
    {
        return;
    }
    sort(delays.begin(), delays.end());
    sort(caps.begin(), caps.end());
    double medDelay = delays[delays.size() / 2];
    double medCap = caps[caps.size() / 2];

    const double delayFactor = 4.0; // delay > 4x median = outlier
    const double capFactor = 4.0;   // capacity < median/4 = outlier
    int fixed = 0;
    for (LinkMeasurement& m : table)
    {
        if (m.baseDelayMs <= 0 || m.baseDelayMs > delayFactor * medDelay)
        {
            cout << "  median-correct " << m.a << "-" << m.b << " delay " << m.baseDelayMs
                 << " -> " << medDelay << " ms" << endl;
            m.baseDelayMs = medDelay;
            ++fixed;
        }
        if (m.capacityBps <= 0 || m.capacityBps < medCap / capFactor)
        {
            cout << "  median-correct " << m.a << "-" << m.b << " cap " << (m.capacityBps / 1e6)
                 << " -> " << (medCap / 1e6) << " Mbps" << endl;
            m.capacityBps = medCap;
            ++fixed;
        }
    }
    cout << "median-correct: " << fixed << " value(s) replaced (medDelay=" << medDelay
         << "ms medCap=" << (medCap / 1e6) << "Mbps)" << endl;
}

void
WriteLinkTable(const vector<LinkMeasurement>& table, const string& path)
{
    ofstream out(path);
    out << "a,b,base_delay_ms,capacity_bps\n";
    for (const LinkMeasurement& m : table)
    {
        out << m.a << ',' << m.b << ',' << m.baseDelayMs << ',' << m.capacityBps << '\n';
    }
    cout << "warm-up: wrote " << table.size() << " links to " << path << endl;
}
