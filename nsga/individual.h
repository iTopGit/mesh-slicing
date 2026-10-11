#ifndef NSGA_INDIVIDUAL_H
#define NSGA_INDIVIDUAL_H

#include <array>
#include <cstdint>
#include <vector>

namespace nsga {

constexpr int NUM_SLICES = 3; // 0 = eMBB, 1 = URLLC, 2 = mMTC

// Objective sense per slice (PopJoi.md): eMBB throughput maximized, URLLC delay
// and mMTC cost minimized. Used by dominance; irrelevant to crowding.
inline constexpr std::array<bool, NUM_SLICES> OBJ_MAXIMIZE = {true, false, false};

// One whole-network routing state: a permutation of each slice's members (the
// evolved genome) plus its evaluation (PopJoi.md "Individual").
struct Individual
{
    std::array<std::vector<uint32_t>, NUM_SLICES> seg; // connect-order per slice
    std::array<double, NUM_SLICES> obj = {0.0, 0.0, 0.0};

    // Members left with no path to their slice's root, summed over the three slices. This is
    // the constraint violation: 0 is feasible, and Dominates() ranks any feasible individual
    // above any infeasible one (Deb's constraint-domination). Without it two of the three
    // objectives REWARD disconnection -- URLLC's delay is the worst member's path delay, which
    // a stranded member contributes nothing to, and mMTC's is its tree's link count, which an
    // empty tree minimises perfectly. "Connect nobody" then scores 0 on both, i.e. the ideal
    // point, which is exactly where KneePoint looks.
    int stranded = 0;

    int rank = 0;
    double crowd = 0.0;
};

} // namespace nsga

#endif // NSGA_INDIVIDUAL_H
