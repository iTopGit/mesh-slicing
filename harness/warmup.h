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
    // Print tx/rx/loss/span per edge in the R window. Capacity is bytes/span, so when it
    // reads wrong this is what says whether the bytes or the span is at fault.
    bool traceR = false;

    /**
     * Bytes delivered between two fixed instants inside one R window, read off the sink.
     * Capacity is this divided by the interval, never FlowMonitor's
     * rxBytes/(timeLastRxPacket - timeFirstRxPacket): the probe overfills the sender's
     * queue on purpose, and the overflow keeps trickling out for seconds afterwards. Those
     * stragglers move timeLastRxPacket without adding bytes, so the span-based figure reads
     * a healthy link several times too slow.
     */
    struct RWindow
    {
        ns3::Ptr<ns3::PacketSink> sink;
        uint64_t startRx = 0;
        uint64_t endRx = 0;
        double dur = 0.0; // <= 0 means no snapshot was taken
    };

    std::vector<RWindow> rwin;
    // PacketSink counts payload only; restate it at the IP layer (+20 IP +8 UDP per packet)
    // so the figure means the same thing as FlowMonitor's rxBytes.
    uint32_t pktSize = 700;
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
    // Drain time between windows. Must exceed the R window's BACKLOG drain, not just the window:
    // a 0.5 s burst above the link rate keeps arriving long after the sender stops, and if that
    // tail laps into the next edge's slot the next edge is measured under contention.
    double guardSec = 0.5;
    bool traceR = false; // per-edge tx/rx/loss/span in the R window; see WarmupContext::traceR
    // R-window offered rate. Only requirement: comfortably above the link ceiling, so the
    // radio is the thing that limits delivery rather than the probe. There is no upper
    // failure mode any more, so the exact value does not matter.
    //
    // It used to matter a great deal, and the reason was a measurement bug, not the rate. A probe
    // above the ceiling always leaves a backlog, and the sweep then carried that backlog into the
    // following links: the leftovers in the queue disc have no lifetime at all, and the ones in the
    // MAC queue are past its 500 ms MaxDelay but are only reaped on the next enqueue, so they sat
    // there reporting the sender busy. A healthy 82.9 Mb/s link read 11.8-35, and the next link
    // from the same sender read nothing at all.
    //
    // What fixed it is resetting the sender between links (FlushSender), which is what RFC 2544
    // asks for -- the device under test returns to a known state between trials. Capacity is then
    // plain rxBytes/(timeLastRxPacket - timeFirstRxPacket) again, because with nothing else on air
    // the backlog drains at the LINK rate, so the drain belongs in the measurement.
    //
    // Measured on 5x5 at a 120 Mb/s probe, three arms (2026-10-07):
    //
    //   arm                          links measured   values patched   bytes/span
    //   edge reordering, no flush        23 / 40            34          10.85-82.78
    //   no reordering, no flush          24 / 40            32          10.83-82.78
    //   no reordering, FLUSH             40 / 40             0          82.63-82.82
    //
    // Reordering the sweep so consecutive links share no node was tried first and did nothing
    // (23 vs 24), so it is not in the code. 10x10 with the flush: 180/180 in one round, none
    // patched, spread 0.43%.
    //
    // The one remaining failure mode is undershoot: at 60 the reading is 62.39, which is the
    // probe itself restated at the IP layer (60 * 728/700), not the link. Silent, so the rate
    // must stay above the ceiling. Removing even that needs a loss-free-rate search in the style
    // of RFC 2544, with the stopping test "measured rate stops following the offered rate" --
    // loss alone does not work, since an 85 Mb/s probe reads the link correctly at 0% loss.
    std::string satRate = "120Mb/s";
    std::string primeRate = "1Mb/s";  // prime rate: low so it resolves ARP without flooding the queue before the delay window
    uint32_t pktSize = 700;           // probe payload; must match the slice traffic or R is measured at the wrong efficiency
    // Filled as the sweep runs:
    // Reset the sender between links instead of hoping its leftovers are gone. Measured on
    // 5x5 at a 120 Mb/s probe: off, 23 of 40 links measured and 34 values needed patching;
    // on, 40 of 40 in one round with none patched. Edge ordering was tried first and did
    // nothing (23 vs 24), so there is no ordering rule here.
    bool flushSender = true;
    // How often the sweep asks whether the last sender's queues are empty. It changes when the
    // next edge is noticed to be clear, never any measured value, so it is not a tuning knob.
    double drainPollSec = 0.01;
    // How long the held-packet count may sit unchanged before the sweep calls the link quiet.
    // Liveness test, not a measurement parameter: it decides when the sweep notices, and any
    // value long enough to outlast one transmission gives the same reading.
    double drainStallSec = 0.2;
    std::uint32_t drainLast = 0xffffffffu;
    double drainLastChange = 0.0;
    std::vector<Edge> edges;
    WarmupContext ctx;
    uint16_t nextPort = 5000;
    double nextTime = 0.0;
    int round = 0;
    // The sweep's work list and position in it; the retry pass refills them.
    std::vector<std::size_t> pending;
    std::size_t cursor = 0;
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
 *
 * Since the sender is reset between links it replaces nothing at 5x5 or 10x10. Treat that as
 * its correct state: it is a backstop for links that never measured, not a filter that healthy
 * readings are expected to need. It also cannot be trusted as a quality signal, since
 * `< median/4` never caught the 35-vs-82.7 split that the old span bug produced.
 */
void MedianCorrectOutliers(std::vector<LinkMeasurement>& table);

/** Write the link table as CSV `a,b,base_delay_ms,capacity_bps` to `path`. */
void WriteLinkTable(const std::vector<LinkMeasurement>& table, const std::string& path);

#endif // WARMUP_H
