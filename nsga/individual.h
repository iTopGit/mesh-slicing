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
    int rank = 0;
    double crowd = 0.0;
};

} // namespace nsga

#endif // NSGA_INDIVIDUAL_H
