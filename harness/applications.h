#ifndef APPLICATIONS_H
#define APPLICATIONS_H

#include "topology.h"

#include <cstdint>

#include "../nsga/individual.h" // nsga::NUM_SLICES
#include "../nsga/slice.h"      // nsga::Slice (shared GA + harness slice)

#include "ns3/ipv4-interface-container.h"

#include <array>

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
 * Returns each slice's SOURCE packet count, which PrintSliceQdiscStats needs: queue-disc
 * counters tally once per hop, so a drop rate divided by enqueues is diluted by the path
 * length -- 33% of URLLC's own packets printed as 10.3% over 3.2 hops.
 */
std::array<std::uint64_t, nsga::NUM_SLICES> PrintSliceResults(
    const std::array<nsga::Slice, nsga::NUM_SLICES>& slices,
    ns3::Ptr<ns3::FlowMonitor> monitor,
    ns3::Ptr<ns3::Ipv4FlowClassifier> classifier);

#endif // APPLICATIONS_H
