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

bool g_obj1Max = false;

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

    for (size_t k = 0; k < perm.size(); ++k)
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

// Links on the unique path between a and b inside one tree. Parent pointers lead to the root,
// so walk a up to the root, then walk b up until it meets that chain: the meeting point is the
// lowest common ancestor and the two partial walks joined are the path. Empty means one endpoint
// never joined the tree, i.e. the pair cannot communicate.
static vector<EdgeKey>
TreePath(const Tree& t, uint32_t a, uint32_t b, uint32_t NONE)
{
    if (a >= t.parent.size() || b >= t.parent.size())
    {
        return {};
    }
    auto ancestors = [&](uint32_t x) {
        vector<uint32_t> up{x};
        while (t.parent[x] != NONE)
        {
            x = t.parent[x];
            up.push_back(x);
        }
        return up;
    };
    vector<uint32_t> upA = ancestors(a);
    vector<uint32_t> upB = ancestors(b);
    if (upA.back() != upB.back()) // different roots -> not in the same tree
    {
        return {};
    }
    // Drop the shared tail; what remains of each walk meets at the lowest common ancestor.
    size_t ia = upA.size(), ib = upB.size();
    while (ia > 1 && ib > 1 && upA[ia - 2] == upB[ib - 2])
    {
        --ia;
        --ib;
    }
    vector<EdgeKey> path;
    for (size_t i = 0; i + 1 < ia; ++i)
    {
        path.push_back(Key(upA[i], upA[i + 1]));
    }
    for (size_t i = 0; i + 1 < ib; ++i)
    {
        path.push_back(Key(upB[i], upB[i + 1]));
    }
    return path;
}

LinkAirtime
SpreadDemand(const Graph& g,
             const array<Tree, NUM_SLICES>& trees,
             const array<Slice, NUM_SLICES>& slices)
{
    const uint32_t NONE = g.NumNodes();
    LinkAirtime a;
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        auto pairs = slices[s].Pairs();
        double perPair = pairs.empty() ? 0.0 : slices[s].demandMbps / pairs.size();
        for (const auto& [x, y] : pairs)
        {
            vector<EdgeKey> path = TreePath(trees[s], x, y, NONE);
            a.pairPaths[s].push_back(path);
            for (const EdgeKey& e : path)
            {
                if (!a.onLink.count(e))
                {
                    a.used.push_back(e);
                    a.onLink[e] = {0, 0, 0};
                }
                a.onLink[e][s] += perPair;
            }
        }
    }
    for (const EdgeKey& e : a.used)
    {
        array<double, NUM_SLICES> D = {0, 0, 0};
        size_t n = 0;
        for (const EdgeKey& f : a.used)
        {
            bool conflicts = g.Hears(e.first, f.first) || g.Hears(e.first, f.second) ||
                             g.Hears(e.second, f.first) || g.Hears(e.second, f.second);
            if (!conflicts)
            {
                continue;
            }
            ++n;
            for (int s = 0; s < NUM_SLICES; ++s)
            {
                D[s] += a.onLink[f][s];
            }
        }
        a.nbhd[e] = D;
        a.nch[e] = n;
    }
    return a;
}

void
Evaluate(Individual& ind,
         const Graph& g,
         const array<Slice, NUM_SLICES>& slices,
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
        double effDelay = 0.0;                       // queueing delay on this link
    };
    map<EdgeKey, LinkLoad> load;
    LinkAirtime air = SpreadDemand(g, trees, slices);
    for (const EdgeKey& e : air.used)
    {
        double R = g.GetLink(e.first, e.second).capacityMbps;
        vector<double> demands;
        vector<int> idx;
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            if (air.nbhd[e][s] > 0.0)
            {
                demands.push_back(air.nbhd[e][s]);
                idx.push_back(s);
            }
        }
        double U = 0.0;
        for (double d : demands)
        {
            U += d / R;
        }
        // The neighbourhood shares one channel of capacity R, so the fair share is taken over
        // the neighbourhood's demands rather than this link's alone.
        vector<double> a = FairShare(R, demands);
        LinkLoad ll;
        for (size_t i = 0; i < idx.size(); ++i)
        {
            ll.alloc[idx[i]] = a[i];
        }
        ll.effDelay = (U < 1.0) ? C / (1.0 - U) : BIG;
        load[e] = ll;
    }

    // Step 3a: the feasibility constraint. A member whose parent pointer is NONE never reached
    // its root, so its traffic goes nowhere. The root itself legitimately has no parent.
    ind.stranded = 0;
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        for (uint32_t m : slices[s].members)
        {
            if (m != trees[s].root && trees[s].parent[m] == NONE)
            {
                ++ind.stranded;
            }
        }
    }

    // Step 3b: reduce each slice's tree to its one objective number.
    // obj[0] eMBB: bottleneck (min) fair-share throughput across its links.
    double embbMin = numeric_limits<double>::infinity();
    for (const vector<EdgeKey>& path : air.pairPaths[0])
    {
        for (const EdgeKey& e : path)
        {
            embbMin = min(embbMin, load[e].alloc[0]);
        }
    }
    ind.obj[0] = isinf(embbMin) ? 0.0 : embbMin; // no eMBB links -> zero throughput

    // obj[1] URLLC: MEAN over pairs of each pair's summed path delay.
    //
    // This was a max over pairs, and that is why Seng's scalar cost beat this one on every metric
    // including this objective: a max prices only the single worst pair, so an individual may
    // pile load onto every other path for free. Seng sums an exponential over all the slice's
    // links and so pays for concentration everywhere. Summing here fixes the same blind spot.
    //
    // Mean rather than sum because the pair count is fixed for an instance, so the two differ
    // only by that constant and rank identically - but the mean stays in milliseconds and can be
    // read against the 10 ms target, while a sum cannot. A pair with no path contributes nothing
    // and is handled by `stranded` through constraint domination.
    double urllcSum = 0.0;
    double urllcWorst = 0.0;
    size_t urllcPairs = 0;
    for (const vector<EdgeKey>& path : air.pairPaths[1])
    {
        double d = 0.0;
        for (const EdgeKey& e : path)
        {
            d += load[e].effDelay;
        }
        urllcSum += d;
        urllcWorst = max(urllcWorst, d);
        ++urllcPairs;
    }
    // g_obj1Max keeps the old max available for the A/B that justifies the change; it is not a
    // policy knob and the default is the repaired form.
    ind.obj[1] = g_obj1Max ? urllcWorst : (urllcPairs ? (urllcSum / urllcPairs) : 0.0);

    // obj[2] mMTC: tree link count (scale/cost — mMTC is low-rate + delay-insensitive,
    // its concern is connecting many nodes cheaply; see PLAN.md §3).
    ind.obj[2] = static_cast<double>(trees[2].links.size());
}

} // namespace nsga
