// NSGA-II backbone driver (PopJoi.md joint design): build the mesh graph, assign
// random disjoint slice members, evolve whole-network routing states, print the
// final Pareto front of (eMBB throughput, URLLC delay, mMTC cost) trade-offs.

#include "fitness.h"
#include "graph.h"
#include "nsga2.h"
#include "routing.h"
#include "seng.h"

#include "ns3/core-module.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>

using namespace ns3;
using namespace nsga;
using namespace std;

// Decode each slice's chromosome into its tree and print every member's route to
// the root (the path its traffic takes). Used to compare initial vs final routing.
static void
PrintPaths(const string& label, const Individual& ind, const Graph& g)
{
    const uint32_t NONE = g.NumNodes();
    const char* names[NUM_SLICES] = {"eMBB ", "URLLC", "mMTC "};
    cout << fixed << setprecision(3);
    cout << "\n=== " << label << " ===\n";
    cout << "  fitness: eMBB_spare=" << ind.obj[0] << "  URLLC_delay=" << ind.obj[1]
         << "  mMTC_links=" << static_cast<int>(ind.obj[2]) << "\n";
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        Tree t = DecodeSlice(g, ind.seg[s]);
        cout << "  " << names[s] << "  root=" << t.root << "  links=" << t.links.size() << "\n";
        for (uint32_t m : ind.seg[s])
        {
            cout << "    " << setw(2) << m << " -> root: ";
            for (uint32_t n = m;; n = t.parent[n])
            {
                cout << n;
                if (n == t.root || t.parent[n] == NONE)
                {
                    break;
                }
                cout << " ";
            }
            cout << "\n";
        }
    }
}

int
main(int argc, char* argv[])
{
    uint32_t gridWidth = 10, gridHeight = 0; // 0 -> square
    uint32_t Me = 0, Mu = 0, Mc = 0; // 0 on all three = derive from the grid, as the harness does
    double embb = 4.0, urllc = 1.0, mmtc = 2.0; // per-slice demand Mb/s
    string linktable;                      // CSV path; empty -> synth grid
    double baseDelayMs = 0.245, capacityMbps = 51.0; // synth-grid link values
    uint32_t popSize = 100, generations = 100, seed = 1;
    double mutationRate = 0.1, t0Ms = 1.1796; // T0: the 10x10 fit; see harness/main.cc
    bool trace = false;
    int showGen = -1;
    bool showPaths = false;
    bool showRoutes = false;
    bool compare = false; // also run the Seng baseline and cross-score both solutions
    std::string obj0 = "spare"; // spare | alloc -- eMBB objective form (alloc is the old one)
    std::string obj1 = "mean";  // mean | max -- URLLC objective over pairs

    CommandLine cmd;
    cmd.AddValue("gridWidth", "Grid width (columns)", gridWidth);
    cmd.AddValue("gridHeight", "Grid height; 0 = square", gridHeight);
    cmd.AddValue("Me", "eMBB member count (0 on all three = derive from the grid)", Me);
    cmd.AddValue("Mu", "URLLC member count (0 on all three = derive from the grid)", Mu);
    cmd.AddValue("Mc", "mMTC member count (0 on all three = derive from the grid)", Mc);
    cmd.AddValue("embb", "eMBB demand Mb/s", embb);
    cmd.AddValue("urllc", "URLLC demand Mb/s", urllc);
    cmd.AddValue("mmtc", "mMTC demand Mb/s", mmtc);
    cmd.AddValue("linktable", "Warm-up link-table CSV (empty = synth grid)", linktable);
    cmd.AddValue("baseDelayMs", "Synth-grid per-link base delay (ms)", baseDelayMs);
    cmd.AddValue("capacityMbps", "Synth-grid per-link capacity (Mb/s)", capacityMbps);
    cmd.AddValue("pop", "Population size N", popSize);
    cmd.AddValue("gens", "Generations", generations);
    cmd.AddValue("mut", "Per-segment swap-mutation probability", mutationRate);
    cmd.AddValue("t0", "T0: per-hop delay (ms) in T = T0/(1-U)", t0Ms);
    cmd.AddValue("seed", "RNG seed", seed);
    cmd.AddValue("trace", "Print per-generation best-objective progress", trace);
    cmd.AddValue("showGen", "Dump one individual at this generation (-1 = never)", showGen);
    cmd.AddValue("showPaths", "Print initial vs final decoded routes", showPaths);
    cmd.AddValue("showRoutes", "Print the knee solution's next-hop routing table", showRoutes);
    cmd.AddValue("obj0", "eMBB objective: spare (default, R(1-U)) or alloc (the old fair share)", obj0);
    cmd.AddValue("compare", "Also run the Seng baseline and print one cross-scored CMP line", compare);
    cmd.AddValue("obj1", "URLLC objective over pairs: mean (default) or max (the old form)", obj1);
    cmd.Parse(argc, argv);
    g_obj0Alloc = (obj0 == "alloc");
    g_obj1Max = (obj1 == "max");

    uint32_t rows = (gridHeight == 0) ? gridWidth : gridHeight;
    uint32_t numNodes = gridWidth * rows;
    // Same quartering rule as the harness, and it has to stay the same: if the two drew different
    // member counts the GA-only sweep and the simulation would be solving different problems.
    if (Me == 0 && Mu == 0 && Mc == 0)
    {
        uint32_t perSlice = numNodes / 4;
        perSlice -= perSlice % 2;
        if (perSlice == 0)
        {
            cout << "error: grid too small to derive member counts (" << numNodes
                 << " nodes gives 0 per slice) - give --Me/--Mu/--Mc explicitly\n";
            return 1;
        }
        Me = Mu = Mc = perSlice;
        cout << "members derived from the grid: " << numNodes << " nodes / 4 = " << perSlice
             << " per slice (even), " << (numNodes - 3 * perSlice) << " left as relay-only\n";
    }
    if (Me + Mu + Mc > numNodes)
    {
        cout << "error: Me+Mu+Mc exceeds mesh size\n";
        return 1;
    }

    Graph g = linktable.empty()
                  ? Graph::FromGrid(gridWidth, rows, baseDelayMs, capacityMbps)
                  : Graph::FromLinkTable(linktable, numNodes);

    // Assign disjoint members: shuffle all node ids, then cut the first Me/Mu/Mc off for
    // eMBB/URLLC/mMTC. This is the fixed problem instance the GA optimizes over. Members are
    // consumed two at a time as source->destination pairs (Slice::Pairs), so the counts are even.
    if (Me % 2 || Mu % 2 || Mc % 2)
    {
        cout << "error: Me/Mu/Mc must be even - members are drawn as src->dst pairs\n";
        return 1;
    }
    mt19937 rng(seed);
    vector<uint32_t> pool(numNodes);
    iota(pool.begin(), pool.end(), 0);
    shuffle(pool.begin(), pool.end(), rng);

    array<Slice, NUM_SLICES> slices;
    slices[0] = {.name = "eMBB", .members = {pool.begin(), pool.begin() + Me}, .demandMbps = embb};
    slices[1] = {.name = "URLLC", .members = {pool.begin() + Me, pool.begin() + Me + Mu}, .demandMbps = urllc};
    slices[2] = {.name = "mMTC", .members = {pool.begin() + Me + Mu, pool.begin() + Me + Mu + Mc}, .demandMbps = mmtc};

    cout << "=== NSGA-II joint routing (" << gridWidth << "x" << rows << " mesh, " << numNodes
              << " nodes) ===\n";
    cout << "source: " << (linktable.empty() ? "synthetic grid" : linktable) << "   pop="
              << popSize << " gens=" << generations << " T0=" << t0Ms << "\n";
    cout << "demand Mb/s:  eMBB=" << embb << "  URLLC=" << urllc << "  mMTC=" << mmtc << "\n\n";

    // Sample one random gen-0 individual (own rng so Run stays reproducible) as the
    // "before" routing, to compare against the evolved knee pick.
    if (showPaths)
    {
        mt19937 initRng(seed);
        Individual init;
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            init.seg[s] = slices[s].members;
            shuffle(init.seg[s].begin(), init.seg[s].end(), initRng);
        }
        Evaluate(init, g, slices, t0Ms);
        PrintPaths("INITIAL — random gen-0 individual", init, g);
    }

    Params params{popSize, generations, mutationRate, t0Ms, trace, showGen};
    vector<Individual> front = Run(g, slices, params, rng);

    // Collapse identical objective vectors: many genomes map to the same trade-off
    // point, so the front's *distinct* points are what matter for the pick.
    sort(front.begin(), front.end(), [](const Individual& a, const Individual& b) {
        if (a.obj[0] != b.obj[0])
        {
            return a.obj[0] > b.obj[0]; // eMBB tp desc
        }
        if (a.obj[1] != b.obj[1])
        {
            return a.obj[1] < b.obj[1]; // URLLC delay asc
        }
        return a.obj[2] < b.obj[2];
    });
    vector<Individual> distinct;
    for (const Individual& ind : front)
    {
        if (distinct.empty() || ind.obj != distinct.back().obj)
        {
            distinct.push_back(ind);
        }
    }

    cout << fixed << setprecision(3);
    cout << "Pareto front F1: " << front.size() << " solutions, " << distinct.size()
              << " distinct trade-off points\n\n";
    size_t knee = KneePoint(distinct);
    cout << "  #  eMBB_spare(Mb/s)^   URLLC_delay(ms)v   mMTC_links v\n";
    for (size_t i = 0; i < distinct.size(); ++i)
    {
        cout << (i == knee ? " *" : "  ") << setw(2) << i << "   " << setw(12) << distinct[i].obj[0]
             << "   " << setw(15) << distinct[i].obj[1] << "   " << setw(10)
             << static_cast<int>(distinct[i].obj[2]) << "\n";
    }
    // Cross-score both optimisers on the SAME problem: each solution gets all three NSGA-II
    // objectives and the Seng cost, so neither method is judged only by its own yardstick.
    // Model-only, no ns-3 - this is what makes a per-seed sweep affordable.
    if (compare)
    {
        mt19937 srng(seed);
        Individual sbest = SengRun(g, slices, params, srng);
        Evaluate(sbest, g, slices, t0Ms);
        const Individual& nk = distinct[knee];
        cout << "CMP seed " << seed << "  nsga spare " << nk.obj[0] << " urllc " << nk.obj[1]
             << " links " << static_cast<int>(nk.obj[2]) << " sengcost " << SengCost(nk, g, slices)
             << "  |  seng spare " << sbest.obj[0] << " urllc " << sbest.obj[1] << " links "
             << static_cast<int>(sbest.obj[2]) << " sengcost " << SengCost(sbest, g, slices) << "\n";
    }

    cout << "\n(^ maximize, v minimize; * = knee-point pick, PLAN.md §6)\n";
    cout << "knee pick: row " << knee << " -> eMBB " << distinct[knee].obj[0] << " Mb/s, URLLC "
         << distinct[knee].obj[1] << " ms, mMTC " << static_cast<int>(distinct[knee].obj[2])
         << " links (balanced compromise, no weights)\n";

    if (showPaths)
    {
        PrintPaths("FINAL — knee-point pick", distinct[knee], g);
    }

    if (showRoutes)
    {
        vector<Route> routes = BuildRoutes(g, distinct[knee], slices);
        sort(routes.begin(), routes.end(), [](const Route& a, const Route& b) {
            if (a.slice != b.slice)
            {
                return a.slice < b.slice;
            }
            if (a.node != b.node)
            {
                return a.node < b.node;
            }
            return a.dest < b.dest;
        });
        const char* sn[NUM_SLICES] = {"eMBB ", "URLLC", "mMTC "};
        cout << "\n=== next-hop routing table (knee solution, " << routes.size() << " entries) ===\n";
        cout << "slice   node   dest   nextHop\n";
        for (const Route& r : routes)
        {
            cout << sn[r.slice] << "  " << setw(4) << r.node << "  " << setw(5) << r.dest << "  "
                 << setw(7) << r.nextHop << "\n";
        }
    }
    return 0;
}
