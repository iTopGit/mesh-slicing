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

    // Links the solution drives to utilization >= 1, i.e. asked to carry more than the medium
    // around them allows. A second constraint, counted rather than priced: it used to be an
    // effDelay of BIG = 1e6 inside obj[1], and that sentinel broke the one property obj[1] needs.
    // obj[1] is otherwise exactly linear in T0, and dominance, crowding (gap/span) and KneePoint
    // all normalise, so scaling obj[1] cannot change which solution is chosen -- T0 is a units
    // label for routing, nothing more. BIG was the single term that did NOT scale with T0, so with
    // one saturated individual in the front the crowding span sat near 1e6 while the healthy gaps
    // grew with T0, and selection shifted: at T0 = 2.6 / 3.9 / 4.53 / 6.0 the same problem
    // installed 272 / 280 / 264 / 272 routes. Counting it restores the invariance.
    int saturated = 0;

    int rank = 0;
    double crowd = 0.0;
};

} // namespace nsga

#endif // NSGA_INDIVIDUAL_H
