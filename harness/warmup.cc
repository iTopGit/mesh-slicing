#include "warmup.h"

#include "ns3/inet-socket-address.h"
#include "ns3/mobility-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>

using namespace ns3;
using namespace std;

// Schedule one edge's isolated sweep at sim time `t`: prime (unmeasured, resolves
// ARP + lets the queue settle so it doesn't corrupt the measurement), then L
// (empty → base_delay), then R (saturating → capacity). One window each, a -> b,
// on fresh ports. Advances s->nextPort; returns the sim time it ends.
static double
ScheduleEdgeWindows(WarmupSession* s, size_t edgeIdx, double t)
{
    const uint32_t pktSize = 700;        // match slice payload (proposal)
    const string lRate = "1Mb/s";        // low → link stays ~empty → base_delay
    const string rRate = s->satRate;     // above link ceiling → saturates → R
    const double primeSec = s->primeSec; // ARP resolve + queue settle before measuring
    const double guard = 0.5;            // drain queue + channel between windows

    uint32_t a = s->edges[edgeIdx].first;
    uint32_t b = s->edges[edgeIdx].second;
    // Apps installed mid-sim initialize at delay 0, and DoInitialize treats the
    // start time as a delay from *now*. So schedule relative to now, not absolute,
    // or a retry installed at t=now would fire at now+now and miss the sim window.
    double now = Simulator::Now().GetSeconds();
    struct SubWindow { string rate; double dur; int phase; }; // phase -1 = prime, unmeasured
    // Prime is low-rate (primeRate): it only needs to resolve ARP. A high-rate prime
    // would flood the MAC queue, and the following L (base-delay) window would then
    // measure the drain, not the link. primeSec <= 0 skips the prime entirely.
    vector<SubWindow> subs;
    if (primeSec > 0.0)
    {
        subs.push_back({s->primeRate, primeSec, -1});
    }
    subs.push_back({lRate, s->windowSec, 0});
    subs.push_back({rRate, s->windowSec, 1});
    for (const SubWindow& sw : subs)
    {
        uint16_t port = s->nextPort++;
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(s->interfaces.GetAddress(b), port));
        onoff.SetConstantRate(DataRate(sw.rate), pktSize);
        ApplicationContainer src = onoff.Install(s->mesh.nodes.Get(a));
        src.Start(Seconds(t - now));
        src.Stop(Seconds(t + sw.dur - now));

        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer rcv = sink.Install(s->mesh.nodes.Get(b));
        rcv.Start(Seconds(t - now));
        rcv.Stop(Seconds(t + sw.dur + guard - now));

        if (sw.phase == 0)
        {
            s->ctx.portToProbe[port] = {edgeIdx, sw.phase};
            s->lastSinks[edgeIdx].first = DynamicCast<PacketSink>(rcv.Get(0));
        }
        else if (sw.phase == 1)
        {
            s->ctx.portToProbe[port] = {edgeIdx, sw.phase};
            s->lastSinks[edgeIdx].second = DynamicCast<PacketSink>(rcv.Get(0));
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

void
BeginWarmup(WarmupSession* s)
{
    s->edges = BuildEdgeListGeometric(s->mesh);
    s->ctx.edges = s->edges;
    cout << "edge list (" << s->edges.size() << " edges), serial sweep" << endl;

    // One edge at a time (staggered): a clean, uncontended base is what the
    // contention model needs. (Spatial-reuse batching was tried and dropped — no
    // real speedup on the mesh, and on wide-channel ax it corrupts the data.)
    double start = Simulator::Now().GetSeconds();
    double t = start;
    for (size_t e = 0; e < s->edges.size(); ++e)
    {
        t = ScheduleEdgeWindows(s, e, t);
    }
    s->nextTime = t;
    s->round = 1;

    // Safety stop bounding the worst case (every round retries every edge). The
    // converging WarmupRetryPass sets an earlier stop, which fires first.
    double perRound = t - start;
    Simulator::Stop(Seconds((t - start) + perRound * (s->maxRounds + 1) + 2.0));

    Simulator::Schedule(Seconds(t - start), &WarmupRetryPass, s);
    cout << "warm-up round 1: " << s->edges.size() << " edges, ends ~" << t << "s" << endl;
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

    double start = Simulator::Now().GetSeconds();
    double t = start;
    // Re-measure just the failed edges, serially, at this later time.
    for (size_t e : bad)
    {
        t = ScheduleEdgeWindows(s, e, t);
    }
    s->nextTime = t;
    ++s->round;
    Simulator::Schedule(Seconds(t - start), &WarmupRetryPass, s);
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
            table[e].capacityBps = (dur > 0) ? (s.rxBytes * 8.0 / dur) : 0.0;
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
