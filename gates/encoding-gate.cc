// PopJoi.md encoding: one individual = 3 concatenated permutations, one per
// slice (eMBB/URLLC/mMTC). Member sets are a fixed problem instance (disjoint
// across slices, chosen once); NSGA-II only ever evolves the *order* within
// each segment. This gate: (1) randomly assigns Me/Mu/Mc disjoint mesh nodes
// as the fixed member sets, (2) builds one individual via P0 init (PopJoi.md
// "random permutation, one per segment"), (3) prints the chromosome.

#include "ns3/core-module.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

using namespace ns3;
using namespace std;

struct Individual
{
    vector<uint32_t> pEmbb;
    vector<uint32_t> pUrllc;
    vector<uint32_t> pMmtc;
};

static void
PrintSeg(const char* name, const vector<uint32_t>& seg)
{
    cout << "  " << name << " [" << seg.size() << "]: ";
    for (uint32_t n : seg)
    {
        cout << n << " ";
    }
    cout << "\n";
}

int
main(int argc, char* argv[])
{
    uint32_t gridWidth = 10, gridHeight = 0; // 0 -> square; 10x10 = full test (max), 5x5 = small test
    uint32_t Me = 10, Mu = 10, Mc = 10;      // members per slice; use 5/5/5 with --gridWidth=5
    uint32_t seed = 1;

    CommandLine cmd;
    cmd.AddValue("gridWidth", "Grid width (columns)", gridWidth);
    cmd.AddValue("gridHeight", "Grid height; 0 = square", gridHeight);
    cmd.AddValue("Me", "eMBB member count", Me);
    cmd.AddValue("Mu", "URLLC member count", Mu);
    cmd.AddValue("Mc", "mMTC member count", Mc);
    cmd.AddValue("seed", "RNG seed", seed);
    cmd.Parse(argc, argv);

    uint32_t rows = (gridHeight == 0) ? gridWidth : gridHeight;
    uint32_t totalNodes = gridWidth * rows;
    if (Me + Mu + Mc > totalNodes)
    {
        cout << "error: Me+Mu+Mc (" << (Me + Mu + Mc) << ") exceeds mesh size (" << totalNodes
             << ")\n";
        return 1;
    }

    mt19937 rng(seed);

    // Fixed problem instance: which nodes belong to which slice (not evolved).
    vector<uint32_t> pool(totalNodes);
    iota(pool.begin(), pool.end(), 0);
    shuffle(pool.begin(), pool.end(), rng);
    vector<uint32_t> embbMembers(pool.begin(), pool.begin() + Me);
    vector<uint32_t> urllcMembers(pool.begin() + Me, pool.begin() + Me + Mu);
    vector<uint32_t> mmtcMembers(pool.begin() + Me + Mu, pool.begin() + Me + Mu + Mc);

    cout << "=== NSGA-II encoding gate (" << gridWidth << "x" << rows << " mesh, " << totalNodes
         << " nodes) ===\n\n";
    cout << "member sets (fixed, disjoint across slices):\n";
    PrintSeg("eMBB ", embbMembers);
    PrintSeg("URLLC", urllcMembers);
    PrintSeg("mMTC ", mmtcMembers);

    // Evolved part: P0 init = random permutation of each segment (PopJoi.md).
    Individual ind{embbMembers, urllcMembers, mmtcMembers};
    shuffle(ind.pEmbb.begin(), ind.pEmbb.end(), rng);
    shuffle(ind.pUrllc.begin(), ind.pUrllc.end(), rng);
    shuffle(ind.pMmtc.begin(), ind.pMmtc.end(), rng);

    cout << "\none individual (chromosome = P_embb ++ P_urllc ++ P_mmtc):\n";
    PrintSeg("P_embb ", ind.pEmbb);
    PrintSeg("P_urllc", ind.pUrllc);
    PrintSeg("P_mmtc ", ind.pMmtc);

    return 0;
}
