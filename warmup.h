#ifndef WARMUP_H
#define WARMUP_H

#include "topology.h"

#include "ns3/flow-monitor.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/packet-sink.h"

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

using Edge = std::pair<uint32_t, uint32_t>;

/** One measured link: endpoints a<b, empty-link one-way delay, saturated capacity. */
struct LinkMeasurement
{
    uint32_t a;
    uint32_t b;
    double baseDelayMs = -1.0;  // L-phase: FlowMonitor delaySum / rxPackets
    double capacityBps = -1.0;  // R-phase: FlowMonitor rxBytes*8 / active time
};

/** Bookkeeping made when probes are scheduled, consumed when results are read. */
struct WarmupContext
{
    std::vector<Edge> edges;
    // dest port -> (index into `edges`, phase: 0 = L/delay, 1 = R/capacity)
    std::map<uint16_t, std::pair<size_t, int>> portToProbe;
};

/**
 * All state a warm-up run carries across its rounds. A probe window can lose every
 * packet to a collision with background control traffic (OLSR) or channel contention
 * (link reads as total loss). Since which edge is hit depends only on *when* its
 * window lands, re-measuring the loser at a later time (a new collision phase)
 * clears it. This holds the round-to-round bookkeeping for that retry loop.
 */
struct WarmupSession
{
    MeshTopology mesh;
    ns3::Ipv4InterfaceContainer interfaces;
    ns3::Ptr<ns3::Ipv4FlowClassifier> classifier;
    ns3::Ptr<ns3::FlowMonitor> monitor;
    double windowSec = 0.5;
    double primeSec = 0.1; // unmeasured pre-roll per edge: resolves ARP + lets the queue settle. <=0 disables it
    int maxRounds = 3;
    std::string satRate = "300Mb/s";  // R-window offered rate; must exceed the ~200 Mb/s ax link ceiling to saturate
    std::string primeRate = "1Mb/s";  // prime rate: low so it resolves ARP without flooding the queue before the delay window
    // Filled as the sweep runs:
    std::vector<Edge> edges;
    WarmupContext ctx;
    uint16_t nextPort = 5000;
    double nextTime = 0.0;
    int round = 0;
    // edge index -> most recent (L sink, R sink); polled between rounds to spot a
    // probe that received nothing, without reading FlowMonitor mid-simulation.
    std::map<size_t, std::pair<ns3::Ptr<ns3::PacketSink>, ns3::Ptr<ns3::PacketSink>>> lastSinks;
};

/**
 * Edge list: enumerate grid-adjacent node pairs from geometry — every i<j within
 * `maxDist` meters. maxDist <= 0 auto-picks (min pairwise distance * 1.3), keeping
 * orthogonal neighbours and dropping diagonals.
 */
std::vector<Edge> BuildEdgeListGeometric(const MeshTopology& mesh, double maxDist = 0.0);

/**
 * Round 1 of the warm-up. Call at sim time `startSec` (via Simulator::Schedule).
 * Enumerates edges, schedules one staggered prime->L->R sweep over all of them,
 * then arms the first WarmupRetryPass to fire once the sweep ends.
 */
void BeginWarmup(WarmupSession* session);

/**
 * Reads results so far, finds edges that lost every packet, and re-measures just
 * those in fresh windows at the current (later) sim time. Reschedules itself
 * until no edge is failing or `maxRounds` is hit, then stops the simulator.
 */
void WarmupRetryPass(WarmupSession* session);

/** After Simulator::Run: read FlowMonitor per-flow stats back into the link table. */
std::vector<LinkMeasurement> ExtractLinkTable(const WarmupContext& ctx,
                                              ns3::Ptr<ns3::Ipv4FlowClassifier> classifier,
                                              ns3::Ptr<ns3::FlowMonitor> monitor);

/**
 * Data-cleanup pass over the measured table (belongs to the GA data-loading step,
 * not the measurement itself): replace any link whose delay/capacity is an extreme
 * outlier vs the sweep median — or that never measured — with the median. All grid
 * links are the same distance, so the true base is near-uniform; a far-off value is
 * a measurement artifact (e.g. the previous edge's saturating window bleeding into
 * this one's queue), not real link variation.
 */
void MedianCorrectOutliers(std::vector<LinkMeasurement>& table);

/** Write the link table as CSV `a,b,base_delay_ms,capacity_bps` to `path`. */
void WriteLinkTable(const std::vector<LinkMeasurement>& table, const std::string& path);

#endif // WARMUP_H
