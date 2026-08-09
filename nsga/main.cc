// NSGA-II backbone driver (PopJoi.md joint design): build the mesh graph, assign
// random disjoint slice members, evolve whole-network routing states, print the
// final Pareto front of (eMBB throughput, URLLC delay, mMTC cost) trade-offs.

#include "graph.h"
#include "nsga2.h"

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
    cout << "  fitness: eMBB_tp=" << ind.obj[0] << "  URLLC_delay=" << ind.obj[1]
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
    uint32_t Me = 10, Mu = 10, Mc = 10;
    double embb = 4.0, urllc = 1.0, mmtc = 2.0; // per-slice demand Mb/s
    string linktable;                      // CSV path; empty -> synth grid
    double baseDelayMs = 0.245, capacityMbps = 51.0; // synth-grid link values
    uint32_t popSize = 100, generations = 100, seed = 1;
    double mutationRate = 0.1, C = 4.53;
    bool trace = false;
    int showGen = -1;
    bool showPaths = false;

    CommandLine cmd;
    cmd.AddValue("gridWidth", "Grid width (columns)", gridWidth);
    cmd.AddValue("gridHeight", "Grid height; 0 = square", gridHeight);
    cmd.AddValue("Me", "eMBB member count", Me);
    cmd.AddValue("Mu", "URLLC member count", Mu);
    cmd.AddValue("Mc", "mMTC member count", Mc);
    cmd.AddValue("embb", "eMBB demand Mb/s", embb);
    cmd.AddValue("urllc", "URLLC demand Mb/s", urllc);
    cmd.AddValue("mmtc", "mMTC demand Mb/s", mmtc);
    cmd.AddValue("linktable", "Warm-up link-table CSV (empty = synth grid)", linktable);
    cmd.AddValue("baseDelayMs", "Synth-grid per-link base delay (ms)", baseDelayMs);
    cmd.AddValue("capacityMbps", "Synth-grid per-link capacity (Mb/s)", capacityMbps);
    cmd.AddValue("pop", "Population size N", popSize);
    cmd.AddValue("gens", "Generations", generations);
    cmd.AddValue("mut", "Per-segment swap-mutation probability", mutationRate);
    cmd.AddValue("C", "Calibrated delay constant (ms)", C);
    cmd.AddValue("seed", "RNG seed", seed);
    cmd.AddValue("trace", "Print per-generation best-objective progress", trace);
    cmd.AddValue("showGen", "Dump one individual at this generation (-1 = never)", showGen);
    cmd.AddValue("showPaths", "Print initial vs final decoded routes", showPaths);
    cmd.Parse(argc, argv);

    uint32_t rows = (gridHeight == 0) ? gridWidth : gridHeight;
    uint32_t numNodes = gridWidth * rows;
    if (Me + Mu + Mc > numNodes)
    {
        cout << "error: Me+Mu+Mc exceeds mesh size\n";
        return 1;
    }

    Graph g = linktable.empty()
                  ? Graph::FromGrid(gridWidth, rows, baseDelayMs, capacityMbps)
                  : Graph::FromLinkTable(linktable, numNodes);

    // Assign disjoint members: shuffle all node ids, then cut the first Me/Mu/Mc off
    // for eMBB/URLLC/mMTC. This is the fixed problem instance the GA optimizes over.
    mt19937 rng(seed);
    vector<uint32_t> pool(numNodes);
    iota(pool.begin(), pool.end(), 0);
    shuffle(pool.begin(), pool.end(), rng);

    array<SliceSpec, NUM_SLICES> slices;
    slices[0] = {{pool.begin(), pool.begin() + Me}, embb};
    slices[1] = {{pool.begin() + Me, pool.begin() + Me + Mu}, urllc};
    slices[2] = {{pool.begin() + Me + Mu, pool.begin() + Me + Mu + Mc}, mmtc};

    cout << "=== NSGA-II joint routing (" << gridWidth << "x" << rows << " mesh, " << numNodes
              << " nodes) ===\n";
    cout << "source: " << (linktable.empty() ? "synthetic grid" : linktable) << "   pop="
              << popSize << " gens=" << generations << " C=" << C << "\n";
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
        Evaluate(init, g, slices, C);
        PrintPaths("INITIAL — random gen-0 individual", init, g);
    }

    Params params{popSize, generations, mutationRate, C, trace, showGen};
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
    cout << "  #   eMBB_tp(Mb/s)^   URLLC_delay(ms)v   mMTC_links v\n";
    for (size_t i = 0; i < distinct.size(); ++i)
    {
        cout << (i == knee ? " *" : "  ") << setw(2) << i << "   " << setw(12) << distinct[i].obj[0]
             << "   " << setw(15) << distinct[i].obj[1] << "   " << setw(10)
             << static_cast<int>(distinct[i].obj[2]) << "\n";
    }
    cout << "\n(^ maximize, v minimize; * = knee-point pick, PLAN.md §6)\n";
    cout << "knee pick: row " << knee << " -> eMBB " << distinct[knee].obj[0] << " Mb/s, URLLC "
         << distinct[knee].obj[1] << " ms, mMTC " << static_cast<int>(distinct[knee].obj[2])
         << " links (balanced compromise, no weights)\n";

    if (showPaths)
    {
        PrintPaths("FINAL — knee-point pick", distinct[knee], g);
    }
    return 0;
}
