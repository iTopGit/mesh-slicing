#ifndef NSGA_OFDMA_ENFORCEMENT_H
#define NSGA_OFDMA_ENFORCEMENT_H

#include "allocator.h"

#include "../nsga/graph.h"
#include "../nsga/fitness.h"
#include "../nsga/individual.h"
#include "../nsga/routing.h"
#include "../nsga/slice.h"

#include <array>
#include <vector>

namespace nsga {
namespace ofdma {

// Control-plane enforcement of the modeled RU split, landed from gates/ofdma-enforce-gate.cc:
// the allocator only decides shares on paper, so a slice that demands more than its share must
// be capped at the SOURCE or it just steals airtime back. ns-3 ad-hoc has no MU scheduler, so
// offered-load control is the only enforcement available (see PLAN.md "OFDMA — modeled
// allocation"). Verifiable end to end: URLLC delay should stay flat as eMBB demand climbs.

// Tightest capacity among the links the chosen routes actually use — the link that constrains
// every slice crossing it, so it is the R the single-link allocator is defined on. Returns 0
// when no route resolves to a known link.
double BottleneckCapacity(const Graph& g, const std::vector<Route>& routes);

// Demand-weighted mean gateway->member hop count over the 3 decoded trees. A slice's
// end-to-end Mb/s costs this many times that in airtime, since every relay retransmits.
// NOT derivable from the route table: BuildRoutes emits an entry at every node of the tree
// for each destination, so routes/destinations is the tree's node count, not its path length.
double AverageHops(const Graph& g,
                   const Individual& ind,
                   const std::array<Slice, NUM_SLICES>& slices);

// Airtime-corrected capacity: the end-to-end rate the mesh can carry, not one link's rate.
// Divides by AverageHops for the relay cost and multiplies by `reuse` for how much of the
// grid transmits at once (1.0 = one collision domain). Measured on a 5x5 ax grid: link R
// was 50 Mb/s and ~11 Mb/s got through end to end.
double EffectiveCapacity(const Graph& g,
                         const Individual& ind,
                         const std::array<Slice, NUM_SLICES>& slices,
                         const std::vector<Route>& routes,
                         double reuse = 1.0);

// SOURCE rates (Mb/s) for --enforce: the allocator's share, clipped so a generator is never
// told to emit more than it wants. Pass the EffectiveCapacity here, not a raw link R, or nothing
// gets capped.
std::array<double, NUM_SLICES> ShapedRates(double C,
                                           const std::array<Slice, NUM_SLICES>& slices,
                                           double urllcHeadroom = 1.64,
                                           double embbFloor = 0.3,
                                           double mmtcHeadroom = 1.0);

} // namespace ofdma
} // namespace nsga

#endif // NSGA_OFDMA_ENFORCEMENT_H
