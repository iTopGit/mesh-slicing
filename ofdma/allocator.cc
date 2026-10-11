// Per-slice rate allocation: cut the mesh's measured capacity into a cap per slice.
// Pure math, ns-3-free -- the single source of truth the gates and the enforcer read.
// Design and the provenance of every constant: Alloc.md §4c.
#include "allocator.h"

#include <algorithm>

using namespace std;

namespace nsga {
namespace ofdma {

array<double, NUM_SLICES>
Allocate(double C,
         const array<double, NUM_SLICES>& demand,
         double urllcHeadroom,
         double embbFloor,
         double mmtcHeadroom)
{
    const int EMBB = 0, URLLC = 1, MMTC = 2;

    // Below 1.0 a "headroom" would hand a slice LESS than it asked for, which is not headroom.
    // 20.0 is the far end (a 5%-utilised lane).
    urllcHeadroom = clamp(urllcHeadroom, 1.0, 20.0);
    mmtcHeadroom = clamp(mmtcHeadroom, 1.0, 20.0);
    embbFloor = clamp(embbFloor, 0.0, 1.0);
    C = max(0.0, C);

    // A floor for a slice with nothing to send would reserve capacity against traffic that does
    // not exist -- the same idle-medium waste the flat-ratio design was rejected for.
    if (demand[EMBB] <= 0.0)
    {
        embbFloor = 0.0;
    }

    // rho* is the fullest any lane may run, and the same figure bounds URLLC's share of the
    // medium. One utilization target used twice -- a reuse, never a derivation (Alloc.md §4c.3).
    const double rhoStar = 1.0 / urllcHeadroom;

    const double su = min(demand[URLLC] * urllcHeadroom, rhoStar * C);
    const double cleft = max(0.0, C - su);
    const double sm = min(demand[MMTC] * mmtcHeadroom, (1.0 - embbFloor) * cleft);
    const double se = max(0.0, cleft - sm);

    array<double, NUM_SLICES> share{};
    share[EMBB] = se;
    share[URLLC] = su;
    share[MMTC] = sm;
    return share;
}

} // namespace ofdma
} // namespace nsga
