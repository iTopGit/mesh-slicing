#ifndef NSGA_FITNESS_H
#define NSGA_FITNESS_H

#include "graph.h"
#include "individual.h"
#include "slice.h"

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace nsga {

// Score obj[1] as the worst pair instead of the mean over pairs. Measurement-only: the max was
// the old form, kept so the repair can be measured against it on one build.
extern bool g_obj1Max;

// Score obj[0] as the old allocated rate instead of bottleneck spare capacity. Measurement-only,
// same role as g_obj1Max: the allocated form is provably degenerate below saturation and is kept
// only so the repair can be measured against it on one build.
extern bool g_obj0Alloc;

// Ordered mesh-edge key (endpoints stored a < b so each link has one key).
using EdgeKey = std::pair<uint32_t, uint32_t>;

// One slice's decoded tree: the mesh links it occupies, a parent pointer per node
// toward the root (first member in the permutation), and the root id. Used for
// fitness — and, for the chosen solution, to walk paths and install real routes.
struct Tree
{
    std::vector<EdgeKey> links;
    std::vector<uint32_t> parent; // parent[n] = next hop toward root; == g.NumNodes() means none
    uint32_t root = 0;
};

// Greedy connect-order decode (PopJoi.md): permutation of members -> tree. Exposed
// so the driver can turn the chosen individual's segments into installable paths.
/**
 * Grow one slice's tree from `perm[0]`, attaching the rest in `perm` order.
 *
 * The root is the chromosome's own first element, so the GA chooses it. That was briefly pinned
 * to `members[0]` because obj[1] summed each member's path TO THE ROOT while the traffic went to
 * `members[0]`, and a root elsewhere made the GA minimise delay to a node no packet was addressed
 * to. Traffic is now fixed member pairs and obj[1] sums each PAIR's path, so nothing depends on
 * which node the tree starts from except the tree's shape - which is exactly what the GA should
 * be free to vary. Pinning it would only remove a degree of freedom for no reason.
 */
Tree DecodeSlice(const Graph& g, const std::vector<uint32_t>& perm);

/**
 * Where each slice's demand actually sits, and how much of the medium each link must share.
 * Shared by the NSGA-II objectives and the Seng baseline so the two score the same physics.
 *
 * `onLink` is the per-pair demand laid along each pair's own tree path - not the slice's full
 * demand on every tree link, which made a funnelled tree indistinguishable from a spread one.
 * `nbhd` sums that over every link whose endpoints can hear this link's endpoints, because one
 * radio channel means a link only transmits while its neighbourhood is silent. `nch` is how many
 * such links there are, which is Seng's `Nch`.
 */
struct LinkAirtime
{
    std::vector<EdgeKey> used;
    std::map<EdgeKey, std::array<double, NUM_SLICES>> onLink;
    std::map<EdgeKey, std::array<double, NUM_SLICES>> nbhd;
    std::map<EdgeKey, std::size_t> nch;
    std::array<std::vector<std::vector<EdgeKey>>, NUM_SLICES> pairPaths;
};

LinkAirtime SpreadDemand(const Graph& g,
                         const std::array<Tree, NUM_SLICES>& trees,
                         const std::array<Slice, NUM_SLICES>& slices);

// Corrected contention model, extracted from formula-gate.cc (PLAN.md §10 step 4).
// Max-min fair share of capacity R across the offered demands: a slice under the
// equal split gets its full ask; leftover re-splits evenly among the rest.
std::vector<double> FairShare(double R, const std::vector<double>& demand);

// Decode the 3 permutations into 3 co-existing trees, overlay joint airtime
// contention per shared link, and write the 3 objectives into ind.obj:
//   obj[0] eMBB  = min SPARE CAPACITY R(1-U) across its pair paths (bottleneck, maximize)
//   obj[1] URLLC = worst member's summed path delay C/(1-U) (minimize)
//   obj[2] mMTC  = tree link count (scale/cost, minimize)
void Evaluate(Individual& ind,
              const Graph& g,
              const std::array<Slice, NUM_SLICES>& slices,
              double t0Ms);

} // namespace nsga

#endif // NSGA_FITNESS_H
