#ifndef NSGA_FITNESS_H
#define NSGA_FITNESS_H

#include "graph.h"
#include "individual.h"

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace nsga {

// One slice's traffic problem: which mesh nodes it spans + how much it pushes.
struct SliceSpec
{
    std::vector<uint32_t> members;
    double demandMbps = 0.0;
};

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
Tree DecodeSlice(const Graph& g, const std::vector<uint32_t>& perm);

// Corrected contention model, extracted from formula-gate.cc (PLAN.md §10 step 4).
// Max-min fair share of capacity R across the offered demands: a slice under the
// equal split gets its full ask; leftover re-splits evenly among the rest.
std::vector<double> FairShare(double R, const std::vector<double>& demand);

// Decode the 3 permutations into 3 co-existing trees, overlay joint airtime
// contention per shared link, and write the 3 objectives into ind.obj:
//   obj[0] eMBB  = min fair-share throughput across its tree (bottleneck, maximize)
//   obj[1] URLLC = worst member's summed path delay C/(1-U) (minimize)
//   obj[2] mMTC  = tree link count (scale/cost, minimize)
void Evaluate(Individual& ind,
              const Graph& g,
              const std::array<SliceSpec, NUM_SLICES>& slices,
              double C);

} // namespace nsga

#endif // NSGA_FITNESS_H
