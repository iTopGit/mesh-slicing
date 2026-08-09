// NSGA-II core (Deb et al. 2002). The whole algorithm, top to bottom:
//   1. make a random start population, score each individual (Evaluate).
//   2. rank everyone by non-domination (which solutions beat which) + crowding
//      (how spread out they are).
//   3. each generation: breed N children (pick parents by tournament -> crossover
//      -> mutation), merge parents+children, keep the best N (best fronts first,
//      ties broken by crowding so the front stays spread out).
//   4. return front 0 = the Pareto set (the best trade-offs).
#include "nsga2.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>

using namespace std;

namespace nsga {

// a "dominates" b when a is no worse on every objective and strictly better on at
// least one. Objectives have different senses (eMBB maximized, the other two
// minimized), so minimized ones are negated to read as "bigger is better" too.
bool
Dominates(const Individual& a, const Individual& b)
{
    bool strictlyBetter = false;
    for (int i = 0; i < NUM_SLICES; ++i)
    {
        double av = a.obj[i];
        double bv = b.obj[i];
        if (!OBJ_MAXIMIZE[i]) // flip so the rest reads as "bigger is better"
        {
            av = -av;
            bv = -bv;
        }
        if (av < bv)
        {
            return false; // worse on this objective
        }
        if (av > bv)
        {
            strictlyBetter = true;
        }
    }
    return strictlyBetter;
}

vector<vector<size_t>>
FastNonDominatedSort(vector<Individual>& pop)
{
    size_t n = pop.size();
    vector<vector<size_t>> dominated(n); // who p dominates
    vector<int> domCount(n, 0);               // how many dominate p
    vector<vector<size_t>> fronts(1);

    // Phase 1: compare every pair. For each p, list who it beats and count how many
    // beat it. Anyone beaten by nobody (domCount 0) is front 0 — the current best.
    for (size_t p = 0; p < n; ++p)
    {
        for (size_t q = 0; q < n; ++q)
        {
            if (p == q)
            {
                continue;
            }
            if (Dominates(pop[p], pop[q]))
            {
                dominated[p].push_back(q);
            }
            else if (Dominates(pop[q], pop[p]))
            {
                domCount[p]++;
            }
        }
        if (domCount[p] == 0)
        {
            pop[p].rank = 0;
            fronts[0].push_back(p);
        }
    }

    // Phase 2: peel off fronts one by one. Conceptually remove the current front;
    // any solution that was only held back by its members now has 0 dominators left,
    // so it becomes the next front. Repeat until everyone is ranked.
    size_t fi = 0;
    while (!fronts[fi].empty())
    {
        vector<size_t> next;
        for (size_t p : fronts[fi])
        {
            // removing this front drops each q's dominator count; hitting 0 = next front
            for (size_t q : dominated[p])
            {
                if (--domCount[q] == 0)
                {
                    pop[q].rank = static_cast<int>(fi + 1);
                    next.push_back(q);
                }
            }
        }
        fi++;
        fronts.push_back(next);
    }
    fronts.pop_back(); // trailing empty
    return fronts;
}

// Crowding distance = how much empty space surrounds a solution on its front.
// Lonely solutions (big gaps to their neighbors) score high and are preferred, so
// the kept front stays spread out instead of clumping. It's the tie-breaker within
// a front when we can't keep everyone.
void
AssignCrowding(vector<Individual>& pop, const vector<size_t>& front)
{
    for (size_t i : front)
    {
        pop[i].crowd = 0.0;
    }
    if (front.size() <= 2) // 1-2 members: no interior, all are "edges" -> always keep
    {
        for (size_t i : front)
        {
            pop[i].crowd = numeric_limits<double>::infinity();
        }
        return;
    }
    // Each objective adds to the score independently: sort the front along it, give
    // the two extremes infinity (never drop a boundary), and every interior solution
    // gains the gap between its left and right neighbors (normalized by the range).
    for (int m = 0; m < NUM_SLICES; ++m)
    {
        vector<size_t> idx(front);
        sort(idx.begin(), idx.end(),
                  [&](size_t a, size_t b) { return pop[a].obj[m] < pop[b].obj[m]; });
        double lo = pop[idx.front()].obj[m];
        double hi = pop[idx.back()].obj[m];
        pop[idx.front()].crowd = numeric_limits<double>::infinity();
        pop[idx.back()].crowd = numeric_limits<double>::infinity();
        double span = hi - lo;
        if (span <= 0.0) // all equal on this objective -> no crowding signal from it
        {
            continue;
        }
        for (size_t k = 1; k + 1 < idx.size(); ++k)
        {
            pop[idx[k]].crowd += (pop[idx[k + 1]].obj[m] - pop[idx[k - 1]].obj[m]) / span;
        }
    }
}

namespace {

// Binary tournament on (rank, crowding): lower rank wins; tie -> larger crowd.
size_t
Tournament(const vector<Individual>& pop, mt19937& rng)
{
    uniform_int_distribution<size_t> pick(0, pop.size() - 1);
    size_t a = pick(rng);
    size_t b = pick(rng);
    if (pop[a].rank != pop[b].rank)
    {
        return (pop[a].rank < pop[b].rank) ? a : b;
    }
    return (pop[a].crowd > pop[b].crowd) ? a : b;
}

// Order crossover (OX) on one permutation: keep a random slice of p1, fill the
// rest from p2 in order. Both are permutations of the same member set.
vector<uint32_t>
OrderCrossover(const vector<uint32_t>& p1,
               const vector<uint32_t>& p2,
               mt19937& rng)
{
    size_t n = p1.size();
    if (n < 2)
    {
        return p1;
    }
    uniform_int_distribution<size_t> pick(0, n - 1);
    size_t i = pick(rng);
    size_t j = pick(rng);
    if (i > j)
    {
        swap(i, j);
    }
    vector<uint32_t> child(n, numeric_limits<uint32_t>::max());
    map<uint32_t, char> inChild;
    for (size_t k = i; k <= j; ++k)
    {
        child[k] = p1[k];
        inChild[p1[k]] = 1;
    }
    size_t pos = (j + 1) % n;
    for (size_t c = 0; c < n; ++c)
    {
        uint32_t gene = p2[(j + 1 + c) % n];
        if (inChild.count(gene))
        {
            continue;
        }
        child[pos] = gene;
        pos = (pos + 1) % n;
    }
    return child;
}

void
SwapMutation(vector<uint32_t>& seg, double rate, mt19937& rng)
{
    if (seg.size() < 2)
    {
        return;
    }
    uniform_real_distribution<double> coin(0.0, 1.0);
    if (coin(rng) >= rate)
    {
        return;
    }
    uniform_int_distribution<size_t> pick(0, seg.size() - 1);
    swap(seg[pick(rng)], seg[pick(rng)]);
}

Individual
MakeChild(const Individual& p1,
          const Individual& p2,
          const Graph& g,
          const array<SliceSpec, NUM_SLICES>& slices,
          const Params& params,
          mt19937& rng)
{
    Individual c;
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        c.seg[s] = OrderCrossover(p1.seg[s], p2.seg[s], rng); // per-segment, disjoint members
        SwapMutation(c.seg[s], params.mutationRate, rng);
    }
    Evaluate(c, g, slices, params.C);
    return c;
}

// --- test tracing: per-generation best of each objective + one-individual dump ---
void
ReportGen(uint32_t gen, const vector<Individual>& pop)
{
    double be = -1e18, bu = 1e18, bm = 1e18; // eMBB max, URLLC min, mMTC min
    for (const Individual& x : pop)
    {
        be = max(be, x.obj[0]);
        bu = min(bu, x.obj[1]);
        bm = min(bm, x.obj[2]);
    }
    cout << "gen " << setw(3) << gen << " |  eMBB_max " << fixed << setprecision(3) << setw(8)
         << be << "   URLLC_min " << setw(10) << bu << "   mMTC_min " << setw(3) << (int)bm << "\n";
}

void
DumpBestUrllc(uint32_t gen, const vector<Individual>& pop)
{
    size_t bi = 0;
    for (size_t i = 1; i < pop.size(); ++i)
    {
        if (pop[i].obj[1] < pop[bi].obj[1])
        {
            bi = i;
        }
    }
    const Individual& ind = pop[bi];
    const char* names[NUM_SLICES] = {"P_embb ", "P_urllc", "P_mmtc "};
    cout << "\n--- one individual at gen " << gen << " (the lowest-URLLC-delay member) ---\n";
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        cout << "  " << names[s] << " [" << ind.seg[s].size() << "]: ";
        for (uint32_t n : ind.seg[s])
        {
            cout << n << " ";
        }
        cout << "\n";
    }
    cout << "  fitness: eMBB_tp=" << fixed << setprecision(3) << ind.obj[0]
         << "  URLLC_delay=" << ind.obj[1] << "  mMTC_links=" << (int)ind.obj[2] << "\n\n";
}

} // namespace

vector<Individual>
Run(const Graph& g,
    const array<SliceSpec, NUM_SLICES>& slices,
    const Params& p,
    mt19937& rng)
{
    // P0: random permutation per segment (PopJoi.md initialization).
    vector<Individual> pop(p.popSize);
    for (Individual& ind : pop)
    {
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            ind.seg[s] = slices[s].members;
            shuffle(ind.seg[s].begin(), ind.seg[s].end(), rng);
        }
        Evaluate(ind, g, slices, p.C);
    }
    auto fronts = FastNonDominatedSort(pop);
    for (const auto& f : fronts)
    {
        AssignCrowding(pop, f);
    }

    // Track the best value each objective has ever reached; note the last gen any of
    // them improved -> everything after that is a plateau (no more growth).
    array<double, NUM_SLICES> best = {-1e18, 1e18, 1e18};
    uint32_t lastImprove = 0;
    auto track = [&](uint32_t gen, const vector<Individual>& population) {
        for (const Individual& x : population)
        {
            if (x.obj[0] > best[0]) { best[0] = x.obj[0]; lastImprove = gen; }
            if (x.obj[1] < best[1]) { best[1] = x.obj[1]; lastImprove = gen; }
            if (x.obj[2] < best[2]) { best[2] = x.obj[2]; lastImprove = gen; }
        }
        if (p.trace)
        {
            ReportGen(gen, population);
        }
        if (p.showGen >= 0 && static_cast<uint32_t>(p.showGen) == gen)
        {
            DumpBestUrllc(gen, population);
        }
    };
    track(0, pop); // generation 0 = the random initial population

    for (uint32_t gen = 0; gen < p.generations; ++gen)
    {
        // Breed N children: each needs 2 parents chosen by tournament (fitter wins),
        // then crossover + mutation inside MakeChild produces + scores the child.
        vector<Individual> offspring;
        offspring.reserve(p.popSize);
        while (offspring.size() < p.popSize)
        {
            const Individual& a = pop[Tournament(pop, rng)];
            const Individual& b = pop[Tournament(pop, rng)];
            offspring.push_back(MakeChild(a, b, g, slices, p, rng));
        }

        // Rt = Pt u Qt, sort, fill Pt+1 by fronts then crowding (elitist mu+lambda).
        vector<Individual> combined = pop;
        combined.insert(combined.end(), offspring.begin(), offspring.end());
        auto cf = FastNonDominatedSort(combined);

        // Take whole fronts best-first while they fit. The one front that overflows N
        // is partially taken, keeping its most crowded-distant (most spread) members.
        vector<Individual> next;
        next.reserve(p.popSize);
        for (const auto& front : cf)
        {
            AssignCrowding(combined, front);
            if (next.size() + front.size() <= p.popSize)
            {
                for (size_t i : front)
                {
                    next.push_back(combined[i]);
                }
            }
            else
            {
                vector<size_t> sorted(front);
                sort(sorted.begin(), sorted.end(), [&](size_t x, size_t y) {
                    return combined[x].crowd > combined[y].crowd;
                });
                for (size_t i : sorted)
                {
                    if (next.size() >= p.popSize)
                    {
                        break;
                    }
                    next.push_back(combined[i]);
                }
                break;
            }
        }
        pop.swap(next);
        track(gen + 1, pop);
    }

    if (p.trace)
    {
        cout << "\n>>> last fitness improvement at gen " << lastImprove << " of " << p.generations
             << " -> growth stops (plateau) after gen " << lastImprove << "\n\n";
    }

    // Evolution done: front 0 of the final population is the Pareto set we return.
    auto finalFronts = FastNonDominatedSort(pop);
    vector<Individual> pareto;
    for (size_t i : finalFronts[0])
    {
        pareto.push_back(pop[i]);
    }
    return pareto;
}

size_t
KneePoint(const vector<Individual>& front)
{
    if (front.size() <= 1)
    {
        return 0;
    }
    // Per-objective range across the front, to normalize onto a common [0,1] scale.
    array<double, NUM_SLICES> lo, hi;
    for (int m = 0; m < NUM_SLICES; ++m)
    {
        lo[m] = hi[m] = front[0].obj[m];
        for (const Individual& x : front)
        {
            lo[m] = min(lo[m], x.obj[m]);
            hi[m] = max(hi[m], x.obj[m]);
        }
    }
    // Closest to the ideal point (0 on every normalized axis) wins. Maximized
    // objectives are flipped so that 0 = best for them too.
    size_t best = 0;
    double bestDist = 1e18;
    for (size_t i = 0; i < front.size(); ++i)
    {
        double d2 = 0.0;
        for (int m = 0; m < NUM_SLICES; ++m)
        {
            double span = hi[m] - lo[m];
            if (span <= 0.0)
            {
                continue;
            }
            double norm = OBJ_MAXIMIZE[m] ? (hi[m] - front[i].obj[m]) / span
                                          : (front[i].obj[m] - lo[m]) / span;
            d2 += norm * norm;
        }
        if (d2 < bestDist)
        {
            bestDist = d2;
            best = i;
        }
    }
    return best;
}

} // namespace nsga
