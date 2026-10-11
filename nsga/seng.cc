#include "seng.h"

#include "fitness.h"
#include "nsga2.h" // OrderCrossover + SwapMutation: both optimisers must vary identically

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>

using namespace std;

namespace nsga {

double
SengCost(const Individual& ind, const Graph& g, const array<Slice, NUM_SLICES>& slices)
{
    array<Tree, NUM_SLICES> trees;
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        trees[s] = DecodeSlice(g, ind.seg[s]);
    }
    LinkAirtime air = SpreadDemand(g, trees, slices);
    double cost = 0.0;
    for (const EdgeKey& e : air.used)
    {
        double bmax = g.GetLink(e.first, e.second).capacityMbps;
        double nch = static_cast<double>(air.nch[e]);
        double used = 0.0;
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            used += air.onLink[e][s];
        }
        // Their equation 3. The over-capacity test is B*Nch > Bmax rather than the paper's
        // literal "B > Bmax", because their own calibration note treats the two as the same
        // condition: "W should have the value 1000 for B = Bmax or B*Nch = Bmax".
        double x = used * nch / bmax;
        if (x > 1.0)
        {
            return numeric_limits<double>::infinity();
        }
        cost += exp(7.0 * x);
    }
    // A solution that strands a member routes nothing for it, which their formulation excludes by
    // construction (a path either exists or the link is unavailable). Keep it unreachable here too.
    if (ind.stranded > 0)
    {
        return numeric_limits<double>::infinity();
    }
    return cost;
}

Individual
SengRun(const Graph& g, const array<Slice, NUM_SLICES>& slices, const Params& p, mt19937& rng)
{
    // Same population size, generations and mutation rate as NSGA-II, so neither method gets
    // more search effort than the other. Selection is Seng's: a tournament of four.
    vector<Individual> pop(p.popSize);
    vector<double> cost(p.popSize);
    for (uint32_t i = 0; i < p.popSize; ++i)
    {
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            pop[i].seg[s] = slices[s].members;
            shuffle(pop[i].seg[s].begin(), pop[i].seg[s].end(), rng);
        }
        Evaluate(pop[i], g, slices, p.C);
        cost[i] = SengCost(pop[i], g, slices);
    }
    auto tournament = [&]() {
        uniform_int_distribution<uint32_t> pick(0, p.popSize - 1);
        uint32_t best = pick(rng);
        for (int k = 1; k < 4; ++k) // tournament size 4, as their Algorithm 1.1
        {
            uint32_t c = pick(rng);
            if (cost[c] < cost[best])
            {
                best = c;
            }
        }
        return best;
    };
    for (uint32_t gen = 0; gen < p.generations; ++gen)
    {
        vector<Individual> next(p.popSize);
        vector<double> nextCost(p.popSize);
        // Elitism: keep the best, or a lucky early solution can be lost and never recovered.
        size_t bi = min_element(cost.begin(), cost.end()) - cost.begin();
        next[0] = pop[bi];
        nextCost[0] = cost[bi];
        for (uint32_t i = 1; i < p.popSize; ++i)
        {
            const Individual& a = pop[tournament()];
            const Individual& b = pop[tournament()];
            Individual c;
            for (int s = 0; s < NUM_SLICES; ++s)
            {
                c.seg[s] = OrderCrossover(a.seg[s], b.seg[s], rng);
                SwapMutation(c.seg[s], p.mutationRate, rng);
            }
            Evaluate(c, g, slices, p.C);
            next[i] = c;
            nextCost[i] = SengCost(c, g, slices);
        }
        pop.swap(next);
        cost.swap(nextCost);
    }
    size_t bi = min_element(cost.begin(), cost.end()) - cost.begin();
    cout << "Seng baseline: best cost " << cost[bi] << " (pop " << p.popSize << ", gens "
         << p.generations << ")\n";
    return pop[bi];
}

} // namespace nsga
