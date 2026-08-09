// Fitness = turn one individual (3 member-orders) into 3 scores. Two stages:
//   decode: each slice's member-order becomes a real tree of mesh links (DecodeSlice)
//   score : lay all 3 trees on the shared mesh, add up airtime contention on each
//           link, then reduce each slice's tree to a single objective (Evaluate).
// The contention math (FairShare for throughput, C/(1-U) for delay) is the model
// checked against ns-3 in formula-gate.cc.
#include "fitness.h"

#include <algorithm>
#include <limits>
#include <map>
#include <utility>

using namespace std;

namespace nsga {

vector<double>
FairShare(double R, const vector<double>& demand)
{
    vector<double> alloc(demand.size(), 0.0);
    vector<char> settled(demand.size(), 0);
    double remaining = R;
    size_t nActive = demand.size();
    bool changed = true;
    // Settle every slice whose demand fits the current equal share; each one settled
    // frees capacity, raising the share for those still over it. Repeat until stable.
    while (changed && nActive > 0)
    {
        changed = false;
        double share = remaining / nActive;
        for (size_t i = 0; i < demand.size(); ++i)
        {
            if (!settled[i] && demand[i] <= share)
            {
                alloc[i] = demand[i];
                remaining -= demand[i];
                settled[i] = 1;
                nActive--;
                changed = true;
            }
        }
    }
    if (nActive > 0)
    {
        // leftover slices each want more than their share -> split the rest evenly
        double share = remaining / nActive;
        for (size_t i = 0; i < demand.size(); ++i)
        {
            if (!settled[i])
            {
                alloc[i] = share;
            }
        }
    }
    return alloc;
}

namespace {

EdgeKey
Key(uint32_t a, uint32_t b)
{
    return (a < b) ? make_pair(a, b) : make_pair(b, a);
}

} // namespace

// Greedy connect-order decode (PopJoi.md): each next member attaches to the nearest
// node already in the tree via the mesh shortest path; relays on that path join too.
Tree
DecodeSlice(const Graph& g, const vector<uint32_t>& perm)
{
    const uint32_t NONE = g.NumNodes();
    Tree t;
    t.parent.assign(g.NumNodes(), NONE);
    if (perm.empty())
    {
        return t;
    }
    vector<char> inSet(g.NumNodes(), 0);
    t.root = perm[0];
    inSet[t.root] = 1;

    for (size_t k = 1; k < perm.size(); ++k)
    {
        uint32_t m = perm[k];
        if (inSet[m])
        {
            continue;
        }
        vector<uint32_t> path = g.NearestInSet(m, inSet); // m .. target(in set)
        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            uint32_t a = path[i];
            uint32_t b = path[i + 1]; // b is one hop closer to the tree/root
            t.links.push_back(Key(a, b));
            if (!inSet[a])
            {
                t.parent[a] = b;
                inSet[a] = 1;
            }
        }
    }
    return t;
}

void
Evaluate(Individual& ind,
         const Graph& g,
         const array<SliceSpec, NUM_SLICES>& slices,
         double C)
{
    const double BIG = 1e6; // U>=1 -> link saturated: a flat huge delay penalty
    const uint32_t NONE = g.NumNodes();

    // Step 1: decode all 3 member-orders into trees (each = a set of mesh links).
    array<Tree, NUM_SLICES> trees;
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        trees[s] = DecodeSlice(g, ind.seg[s]);
    }

    // Step 2: overlay contention. For each link any tree touches, collect the demands
    // of the slices sharing it -> total airtime U -> per-slice fair-share rate and a
    // shared queueing delay. `uses` marks which slices ride each link; `load` holds
    // the computed result per link.
    struct LinkLoad
    {
        array<double, NUM_SLICES> alloc = {0, 0, 0}; // fair-share throughput per slice
        double effDelay = 0.0;                        // queueing delay on this link
    };
    map<EdgeKey, LinkLoad> load;
    map<EdgeKey, array<char, NUM_SLICES>> uses;

    for (int s = 0; s < NUM_SLICES; ++s)
    {
        for (const EdgeKey& e : trees[s].links)
        {
            uses[e][s] = 1;
        }
    }
    for (auto& [e, who] : uses)
    {
        double R = g.GetLink(e.first, e.second).capacityMbps;
        vector<double> demands;
        vector<int> idx;
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            if (who[s])
            {
                demands.push_back(slices[s].demandMbps);
                idx.push_back(s);
            }
        }
        double U = 0.0;
        for (double d : demands)
        {
            U += d / R;
        }
        vector<double> a = FairShare(R, demands);
        LinkLoad ll;
        for (size_t i = 0; i < idx.size(); ++i)
        {
            ll.alloc[idx[i]] = a[i];
        }
        ll.effDelay = (U < 1.0) ? C / (1.0 - U) : BIG;
        load[e] = ll;
    }

    // Step 3: reduce each slice's tree to its one objective number.
    // obj[0] eMBB: bottleneck (min) fair-share throughput across its links.
    double embbMin = numeric_limits<double>::infinity();
    for (const EdgeKey& e : trees[0].links)
    {
        embbMin = min(embbMin, load[e].alloc[0]);
    }
    ind.obj[0] = isinf(embbMin) ? 0.0 : embbMin; // no eMBB links -> zero throughput

    // obj[1] URLLC: worst member's summed path-to-root delay.
    double urllcMax = 0.0;
    for (uint32_t m : slices[1].members)
    {
        // walk parent pointers from this member up to the root, summing each hop's delay
        double d = 0.0;
        for (uint32_t n = m; trees[1].parent[n] != NONE; n = trees[1].parent[n])
        {
            d += load[Key(n, trees[1].parent[n])].effDelay;
        }
        urllcMax = max(urllcMax, d);
    }
    ind.obj[1] = urllcMax;

    // obj[2] mMTC: tree link count (scale/cost proxy — PopJoi.md TBD).
    ind.obj[2] = static_cast<double>(trees[2].links.size());
}

} // namespace nsga
