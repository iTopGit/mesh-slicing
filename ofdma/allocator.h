#ifndef NSGA_OFDMA_ALLOCATOR_H
#define NSGA_OFDMA_ALLOCATOR_H

#include "../nsga/individual.h" // NUM_SLICES + slice-index convention (0=eMBB,1=URLLC,2=mMTC)

#include <array>

namespace nsga {
namespace ofdma {

// Per-slice rate allocation, Alloc.md §4c.2. Four steps, served in priority order
// (Bertsekas & Gallager §3.5.3 numbering, class 1 = highest):
//
//   1. cap_1 = min( lambda_1 * Headroom,  rho* * C )      URLLC
//   2. Cleft = C - cap_1
//   3. cap_2 = min( lambda_2 * mmtcHeadroom, (1-F) * Cleft )  mMTC
//   4. cap_3 = Cleft - cap_2                              eMBB, residual
//
// Every cap is min-ed against the slice's own need, so none can bind while capacity is
// spare -- the work-conserving property `alloc-gate` case 5 measured (a clamped cap gave
// mMTC 1.02 Mb/s and 2197 ms on a medium that never congested).
//
// Per-slice rationale:
//   URLLC (1): lane sized so its utilization sits at rho*, not 1. Its ceiling is rho* * C --
//              without it a demand-derived reservation is a blank cheque, measured as URLLC
//              taking 39.79 Mb/s while eMBB got 1.46 and mMTC 0.53.
//   mMTC  (2): headroom 1.0. Nerini & Palma model it as constant-rate with no retransmission,
//              and a source that cannot exceed its own rate needs no headroom; clumping is the
//              token bucket's burst, not its rate. See Alloc.md §4c.5.
//   eMBB  (0): elastic residual, floored at F * Cleft by capping mMTC at (1-F) * Cleft.
//
// `C` is the MESH end-to-end capacity from the AIMD controller, NOT the per-link warm-up rate
// (`C_link`); they differ by about 7x and passing the wrong one makes every cap too generous.
std::array<double, NUM_SLICES> Allocate(double C,
                                        const std::array<double, NUM_SLICES>& demand,
                                        double urllcHeadroom = 1.64,
                                        double embbFloor = 0.3,
                                        double mmtcHeadroom = 1.0);

} // namespace ofdma
} // namespace nsga

#endif // NSGA_OFDMA_ALLOCATOR_H
