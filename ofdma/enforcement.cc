#include "enforcement.h"

#include <algorithm>
#include <set>
#include <limits>
#include <stdexcept>

using namespace std;

namespace nsga {
namespace ofdma {

double
BottleneckCapacity(const Graph& g, const vector<Route>& routes)
{
    double bottleneck = numeric_limits<double>::max();
    bool found = false;
    for (const Route& r : routes)
    {
        // Routes come from tree edges so the link exists; guard anyway since a caller could
        // pass routes built on a different graph (Graph::GetLink throws on a missing key).
        double cap = 0.0;
        try
        {
            cap = g.GetLink(r.node, r.nextHop).capacityMbps;
        }
        catch (const out_of_range&)
        {
            continue;
        }
        if (cap <= 0.0) // unmeasured edge
        {
            continue;
        }
        bottleneck = min(bottleneck, cap);
        found = true;
    }
    return found ? bottleneck : 0.0;
}

double
AverageHops(const Graph& g, const Individual& ind, const array<Slice, NUM_SLICES>& slices)
{
    const uint32_t NONE = g.NumNodes();
    // Weight by demand: the heavy slice's path length is what actually burns airtime.
    double weighted = 0.0;
    double totalDemand = 0.0;
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        Tree t = DecodeSlice(g, ind.seg[s]);
        double hops = 0.0;
        int counted = 0;
        for (uint32_t m : slices[s].members)
        {
            if (m == t.root || m >= t.parent.size())
            {
                continue;
            }
            double h = 0.0;
            for (uint32_t n = m; t.parent[n] != NONE; n = t.parent[n])
            {
                ++h;
            }
            hops += h;
            ++counted;
        }
        if (counted == 0 || slices[s].demandMbps <= 0.0)
        {
            continue;
        }
        weighted += slices[s].demandMbps * (hops / counted);
        totalDemand += slices[s].demandMbps;
    }
    return (totalDemand > 0.0) ? weighted / totalDemand : 0.0;
}

double
EffectiveCapacity(const Graph& g,
                  const Individual& ind,
                  const array<Slice, NUM_SLICES>& slices,
                  const vector<Route>& routes,
                  double reuse)
{
    double R = BottleneckCapacity(g, routes);
    double hops = AverageHops(g, ind, slices);
    if (R <= 0.0 || hops <= 0.0)
    {
        return 0.0;
    }
    return R * max(reuse, 1.0) / hops;
}


// The raw per-slice share: URLLC reserved with headroom, mMTC next, eMBB the residual with a floor
// on what is LEFT rather than on capacity.
//
// Internal, and the reason it is separate from ShapedRates is worth keeping. A SOURCE cap above
// what the source wants is pointless, so ShapedRates clips this to demand. A BUCKET rate above
// what the slice sends is exactly the spare room that keeps its queue draining -- clip it and the
// lane runs at utilization 1, where any arrival clump leaves a backlog that never recovers however
// large the bucket is. The two were one function until 2026-09-16, which silently cancelled
// urllcHeadroom on the qdisc path: URLLC's bucket sat at precisely its own demand and policed
// 0.92% of its own traffic.
//
// Note the qdisc path no longer comes through here at all - the per-node controller calls
// ofdma::Allocate directly (harness/controller.cc), which applies the same ordering without
// clipping. So this is now ShapedRates' first step and nothing else.
static array<double, NUM_SLICES>
SliceCaps(double C,
          const array<Slice, NUM_SLICES>& slices,
          double urllcHeadroom,
          double embbFloor,
          double mmtcHeadroom)
{
    array<double, NUM_SLICES> share{};
    double urllcNeed = slices[1].demandMbps * urllcHeadroom;
    share[1] = min(urllcNeed, C);
    double left = max(C - share[1], 0.0);
    share[2] = min(slices[2].demandMbps * mmtcHeadroom, left);
    left = max(left - share[2], 0.0);
    share[0] = max(left, embbFloor * max(C - share[1], 0.0));
    return share;
}

array<double, NUM_SLICES>
ShapedRates(double C,
            const array<Slice, NUM_SLICES>& slices,
            double urllcHeadroom,
            double embbFloor,
            double mmtcHeadroom)
{
    array<double, NUM_SLICES> share =
        SliceCaps(C, slices, urllcHeadroom, embbFloor, mmtcHeadroom);
    array<double, NUM_SLICES> onWire{};
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        onWire[s] = min(slices[s].demandMbps, share[s]);
    }
    return onWire;
}

} // namespace ofdma
} // namespace nsga
