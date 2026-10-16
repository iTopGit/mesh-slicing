#ifndef NSGA_OFDMA_ALLOCATOR_H
#define NSGA_OFDMA_ALLOCATOR_H

#include "../nsga/individual.h" // NUM_SLICES + slice-index convention (0=eMBB,1=URLLC,2=mMTC)

#include <array>
#include <cstdint>

namespace nsga {
namespace ofdma {

// Per-slice rate allocation, Alloc.md §4c.2. Four steps, served in priority order
// (Bertsekas & Gallager §3.5.3 numbering, class 1 = highest):
//
//   1. cap_1 = min( lambda_1 * Headroom,  rho* * admitBudgetMbps )      URLLC
//   2. Cleft = admitBudgetMbps - cap_1
//   3. cap_2 = min( lambda_2 * mmtcHeadroom, (1-F) * Cleft )  mMTC
//   4. cap_3 = Cleft - cap_2                              eMBB, residual
//
// Every cap is min-ed against the slice's own need, so none can bind while capacity is
// spare -- the work-conserving property `alloc-gate` case 5 measured (a clamped cap gave
// mMTC 1.02 Mb/s and 2197 ms on a medium that never congested).
//
// Per-slice rationale:
//   URLLC (1): lane sized so its utilization sits at rho*, not 1. Its ceiling is rho* * admitBudgetMbps --
//              without it a demand-derived reservation is a blank cheque, measured as URLLC
//              taking 39.79 Mb/s while eMBB got 1.46 and mMTC 0.53.
//   mMTC  (2): headroom 1.2, DECLARED not derived. The earlier 1.0 argued a constant-rate source
//              needs no headroom, which ignores that a cap equal to the demand puts the lane at
//              utilization 1, where queueing delay is unbounded whatever the source does. 1.2 is a
//              scope decision: mMTC is out of this thesis's focus, no source paper sets it a delay
//              or loss target, and the only claim made for the number is rho < 1 so the lane
//              drains. See Alloc.md §4c.5 and harness/main.cc.
//   eMBB  (0): elastic residual, floored at F * Cleft by capping mMTC at (1-F) * Cleft.
//
// `admitBudgetMbps` is the MESH end-to-end capacity from the AIMD controller, NOT the per-link warm-up rate
// (`C_link`); they differ by about 7x and passing the wrong one makes every cap too generous.
std::array<double, NUM_SLICES> Allocate(double admitBudgetMbps,
                                        const std::array<double, NUM_SLICES>& demand,
                                        double urllcHeadroom,
                                        double embbFloor = 0.3,
                                        double mmtcHeadroom = 1.2,
                                        bool secondRound = true);

} // namespace ofdma
} // namespace nsga

#endif // NSGA_OFDMA_ALLOCATOR_H
