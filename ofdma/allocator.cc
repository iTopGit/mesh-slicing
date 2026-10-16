// Per-slice rate allocation: cut the mesh's measured capacity into a cap per slice.
// Pure math, ns-3-free -- the single source of truth the gates and the enforcer read.
// Design and the provenance of every constant: Alloc.md §4c.
#include "allocator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace std;

namespace nsga {
namespace ofdma {

array<double, NUM_SLICES>
Allocate(double admitBudgetMbps,
         const array<double, NUM_SLICES>& demand,
         double urllcHeadroom,
         double embbFloor,
         double mmtcHeadroom,
         bool secondRound)
{
    const int EMBB = 0, URLLC = 1, MMTC = 2;

    // Below 1.0 a "headroom" would hand a slice LESS than it asked for, which is not headroom.
    // Rejected rather than clamped up: 0 means the caller never derived it, and silently turning
    // that into 1.0 would allocate URLLC exactly its demand -- utilization 1, the defect
    // capacity-feedback-gate already measured on mMTC. 20.0 is the far end (a 5%-utilised lane).
    if (!(urllcHeadroom >= 1.0))
    {
        throw invalid_argument("ofdma::Allocate: urllcHeadroom must be >= 1.0 and derived, not 0");
    }
    urllcHeadroom = min(urllcHeadroom, 20.0);
    mmtcHeadroom = clamp(mmtcHeadroom, 1.0, 20.0);
    embbFloor = clamp(embbFloor, 0.0, 1.0);
    admitBudgetMbps = max(0.0, admitBudgetMbps);

    // A floor for a slice with nothing to send would reserve capacity against traffic that does
    // not exist -- the same idle-medium waste the flat-ratio design was rejected for.
    if (demand[EMBB] <= 0.0)
    {
        embbFloor = 0.0;
    }

    // rho* is the fullest any lane may run, and the same figure bounds URLLC's limit of the
    // medium. One utilization target used twice -- a reuse, never a derivation (Alloc.md §4c.3).
    const double rhoStar = 1.0 / urllcHeadroom;

    const double su = min(demand[URLLC] * urllcHeadroom, rhoStar * admitBudgetMbps);
    const double bleft = max(0.0, admitBudgetMbps - su);
    const double sm = min(demand[MMTC] * mmtcHeadroom, (1.0 - embbFloor) * bleft);
    // eMBB is capped by its OWN need too, 2026-10-16. It used to take the whole residual, which
    // made it the only unbounded lane: measured at load 12, eMBB held 71.19 Mb/s per node against
    // URLLC's 5.56 -- 12.8x -- on 2.44 offered. The reservation ORDER then meant nothing, because
    // the slice served last held the largest limit, contended hardest and won the medium. Measured
    // at load 27: eMBB lost 31% while URLLC, the slice the order exists to protect, lost 66%.
    //
    // The residual design assumes the budget is a private pool that must be spent. That is true of
    // a wire and false of a shared medium: budget left unallocated here is not idle capacity, it is
    // AIRTIME LEFT FOR THE NEIGHBOURS. Handing it to eMBB does not let eMBB use spare capacity, it
    // lets eMBB contend harder, and the cost lands on URLLC.
    //
    // So all three lanes are now bounded by their own need -- but NOT at the same headroom, and
    // that asymmetry is the point (user, 2026-10-16). Headroom buys one thing: it keeps a lane's
    // utilization below 1 so a burst waits less. eMBB is the slice with no delay requirement, so
    // it is the one lane that does not need to buy that. Its cap is therefore its RAW demand,
    // utilization 1, and its congestion shows up as loss rather than as waiting -- which is where
    // the design wants it, and it leaves the most air for URLLC. Giving eMBB URLLC's 1/rho* was
    // tried first and is more generous than anything eMBB can justify.
    // Note what this retires: `embbFloor` (F) reserved eMBB a share of what URLLC left, which is
    // moot once eMBB cannot exceed its own need -- F now only limits mMTC. Say so before quoting
    // any F sweep taken before this date.
    const double se = min(demand[EMBB], max(0.0, bleft - sm));

    array<double, NUM_SLICES> limit{};
    limit[EMBB] = se;
    limit[URLLC] = su;
    limit[MMTC] = sm;

    if (!secondRound)
    {
        return limit;
    }

    // SECOND ROUND. Pass 1 hands every leftover to eMBB (`se = bleft - sm`), so a slice held at its
    // ceiling cannot touch spare capacity even when it is the one starving. Nothing is wasted -
    // the three shares always sum to capacity - but the spare always lands on the elastic slice.
    //
    // So: reclaim what a slice will not use, and offer it in the design's own priority order. Each
    // slice is capped at its NEED, not its raw demand -- `demand * headroom`, the same quantity
    // pass 1 asks for. Trimming to raw demand would set a cap equal to arrival rate, which is
    // utilization 1 and unbounded queueing: the defect capacity-feedback-gate measured on mMTC.
    //
    // It cannot manufacture the "URLLC degrades last" result, because under real scarcity every
    // slice wants more than its limit, nothing is reclaimed, and this whole block is inert. It acts
    // only where spare exists. The demand cap is what stops it becoming the blank cheque alloc-gate
    // measured (URLLC demanding 40 took 39.79 Mb/s while eMBB got 1.46 at 89.3% loss).
    //
    // OFF BY DEFAULT since 2026-10-14, kept as a measured arm. Measured both ways at 10x10, F = 0.3:
    //   asymmetric 2/20/2 - the case it exists for - URLLC 359 -> 103 ms, 1.20% -> 0% loss, every
    //                       packet delivered. A clear win.
    //   symmetric 12/12/12 - URLLC 409 -> 581 ms, 1.38% -> 2.87% loss. A clear cost.
    // The cost is structural, not a tuning miss: `need` is sized from demand measured in the LAST
    // control tick, so on a node that was quiet it reclaims nearly everything -- taking exactly the
    // slack that would have absorbed the NEXT burst. More rounds, or a finer resource-unit
    // round-robin, reclaim more often and make it worse. The only thing that survives is a
    // reservation INDEPENDENT of measured demand, which is what `rho* x capacity` already is in
    // pass 1. So the design had the right structure and redistribution erodes it.
    // Every lane is held to the SAME utilization target, which is what rho* already means here --
    // "the fullest any lane may run". So eMBB's need carries the same headroom as URLLC's.
    //
    // Fixed 2026-10-14: this first read `demand[EMBB]` with no headroom, which trimmed eMBB to a cap
    // equal to its arrival rate -- utilization 1, unbounded queueing, the defect
    // capacity-feedback-gate measured on mMTC. It survived an asymmetric test where eMBB's demand was
    // 2 Mb/s and then made EVERY run INVALID at 12 Mb/s per slice: three F values, three runs that
    // never drained inside the 120 s cap, against one-pass runs that drained in 11-14 s.
    // Each entry must be the SAME quantity pass 1 caps that lane at, or round 2 hands back more
    // than pass 1 allowed. eMBB's was left at `/ rhoStar` when pass 1 moved to raw demand on
    // 2026-10-16, which would have given eMBB twice its pass-1 cap; corrected the same day.
    const array<double, NUM_SLICES> need = {demand[EMBB],
                                            demand[URLLC] / rhoStar,
                                            demand[MMTC] * mmtcHeadroom};
    double spare = 0.0;
    for (int i = 0; i < NUM_SLICES; ++i)
    {
        if (limit[i] > need[i])
        {
            spare += limit[i] - need[i];
            limit[i] = need[i];
        }
    }
    for (int i : {URLLC, MMTC, EMBB})
    {
        if (spare <= 0.0)
        {
            break;
        }
        const double give = min(max(0.0, need[i] - limit[i]), spare);
        limit[i] += give;
        spare -= give;
    }
    // Whatever nobody claimed goes back to eMBB. An unallocated cap would only idle the medium, and
    // eMBB is the slice with no delay requirement, so an over-generous cap costs it nothing.
    limit[EMBB] += spare;
    return limit;
}

} // namespace ofdma
} // namespace nsga
