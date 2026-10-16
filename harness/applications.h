#ifndef APPLICATIONS_H
#define APPLICATIONS_H

#include "topology.h"

#include <cstdint>

#include "../nsga/individual.h" // nsga::NUM_SLICES
#include "../nsga/slice.h"      // nsga::Slice (shared GA + harness slice)

#include "ns3/ipv4-interface-container.h"

#include <array>
#include <map>
#include <string>
#include <vector>

/**
 * Install a slice's traffic on the mesh: one UDP flow per member PAIR (Slice::Pairs - consecutive
 * couples of the member list), ToS-marked (slice.tos) and rate-shaped so the flows sum to
 * `onWireMbps`. The pairs are the slice's demand matrix and are fixed before the GA runs, so every
 * individual is scored on the same flows; see Slice::Pairs for why they must not come from the
 * chromosome. This replaced a star to members[0], which put the whole slice's load on one node's
 * first link in every run and so never exercised parallel or crossed routes.
 *
 * `bidirectional` adds the matching downlink (gateway -> member) and is OFF by default, because
 * every source in `source/` measures one direction only: Seng et al. state they use one-way UDP
 * to "measure the maximum possible rate in one direction", the 5G Wi-Fi slicing paper takes "the
 * uplink communication... unlike previous studies which focus only on the downlink scheduling",
 * and Smart Resource Allocation sends "from the AP to STAs". One-way is therefore the arm that
 * compares against them. Two-way is the harder, more realistic arm - it loads the half-duplex
 * relay from both sides and makes the gateway a hotspot in both - and is reported separately,
 * never mixed into the same figure.
 *
 * Ports always span [slice.port, slice.port + 2*(members-1)) whichever direction count is used,
 * so the per-slice aggregation range in PrintSliceResults does not shift between arms.
 * Needs routes already in place (GA-injected or OLSR) — the flows are multi-hop.
 *
 * `onWireMbps` < slice.demandMbps IS the OFDMA enforcement: the excess never reaches the
 * wire, freeing airtime for the other slices (ofdma::ShapedRates). Pass slice.demandMbps
 * to run unenforced.
 */
void InstallSliceTraffic(const MeshTopology& mesh,
                         const ns3::Ipv4InterfaceContainer& interfaces,
                         const nsga::Slice& slice,
                         double onWireMbps,
                         bool bidirectional,
                         double startSec,
                         double stopSec,
                         uint32_t pktSize = 700);

// Forward-declared so this header stays light; the impl pulls the flow-monitor module.
namespace ns3
{
class FlowMonitor;
class Ipv4FlowClassifier;
} // namespace ns3

/**
 * Aggregate FlowMonitor stats per slice (flows grouped by their port range) and print one
 * line per slice: throughput / delay / loss. Call after the traffic run.
 *
 * `trafficSec` is the window the sources were actually sending for, and it is what goodput is
 * divided by. It has no default on purpose: the arrival span was the old divisor and it made two
 * identical workloads print different throughput, so a divisor that can be left unstated is the
 * defect waiting to come back. See the comment on `thpt` in applications.cc.
 *
 * Returns each slice's SOURCE packet count, which PrintSliceQdiscStats needs: queue-disc
 * counters tally once per hop, so a drop rate divided by enqueues is diluted by the path
 * length -- 33% of URLLC's own packets printed as 10.3% over 3.2 hops.
 */
/**
 * Per-flow counters captured at the moment the measurement window OPENS, so the window's figures
 * can be differenced out of FlowMonitor's run-long totals.
 *
 * Why a window at all: the first seconds of traffic are the initial transient, not the system
 * under test. The AIMD admit budget starts at `admitBudgetMax` and walks down (16 ticks in the
 * old 8 s window, and measured still unsettled at both 5 and 9 Mb/s per slice), and the queues
 * start empty and fill - at 9 Mb/s per slice the FIFO arm's drain took 5.5-20.3 s, i.e. LONGER
 * than the whole old window, so those runs measured the filling rather than the filled state.
 * Worse, the arms settle at different rates (0.1-0.8 s with Cobalt against 5.5-20.3 s with the
 * Fifo child), so the transient is a confound between arms, not a shared offset.
 */
struct FlowBase
{
    std::uint64_t txPackets = 0;
    std::uint64_t rxPackets = 0;
    std::uint64_t rxBytes = 0;
    double delaySum = 0.0;
};

struct FlowWindow
{
    std::map<std::uint32_t, FlowBase> base; // per FlowId, as of openSec
    double openSec = -1.0; // < 0 = never opened
    // Free-text occupancy note, printed with the window. Only ONE window per run can read the
    // watcher (the call stops it), so the rest carry "" and print nothing rather than asserting
    // something never measured -- CLAUDE.md rule 4.
    std::string note;
    // Held-packet count when the window opened and when it closed. A steady-state window ends
    // about where it started; one that ends far higher contained net ACCUMULATION, so its figures
    // are a transient. This is a coarse TRIPWIRE for a pathological run, not a precision test --
    // if a 40 s run cannot settle, the fault is in the code or the configuration, not in the
    // window length (user, 2026-10-15).
    std::uint32_t heldOpen = 0;
    std::uint32_t heldClose = 0;
    bool closed = false;
};

/** Record the held count as the window closes and print the settle tripwire verdict. */
void CloseFlowWindow(const MeshTopology& mesh, FlowWindow& w, double growthFactor);

/** True if every closed window ended near where it opened. False = the run measured a transient. */
bool WindowsSettled(const std::vector<FlowWindow>& windows, double growthFactor);

/** Capture every flow's counters into `w` and mark the window open. */
void OpenFlowWindow(const MeshTopology& mesh,
                    ns3::Ptr<ns3::FlowMonitor> monitor,
                    FlowWindow& w,
                    const std::string& note);

std::array<std::uint64_t, nsga::NUM_SLICES> PrintSliceResults(
    const std::array<nsga::Slice, nsga::NUM_SLICES>& slices,
    ns3::Ptr<ns3::FlowMonitor> monitor,
    ns3::Ptr<ns3::Ipv4FlowClassifier> classifier,
    double trafficSec,
    const FlowWindow* window = nullptr);

#endif // APPLICATIONS_H
