#ifndef NSGA_SLICE_H
#define NSGA_SLICE_H

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace nsga {

// One network slice — the single source of truth shared by the GA and the harness.
// The GA reads `members` + `demandMbps`; the harness additionally uses `tos` + `port`
// to generate and classify the slice's traffic. Replaces the old split between
// nsga::SliceSpec (members+demand) and the harness's own single-pair Slice.
struct Slice
{
    std::string name;                   // "eMBB" / "URLLC" / "mMTC"
    std::vector<std::uint32_t> members; // mesh nodes this slice spans
    double demandMbps = 0.0;            // total offered load across the slice
    std::uint8_t tos = 0;               // slice marking (SliceTos); harness-only
    std::uint16_t port = 0;             // base UDP port for its flows; harness-only

    /**
     * The slice's flows: consecutive member couples, so `members = [a,b,c,d]` means one flow
     * `a -> b` and one `c -> d`. The member draw is a shuffle, so consecutive couples are
     * already a random matching; nothing further is randomised here.
     *
     * These pairs are the slice's DEMAND MATRIX and are fixed before the GA runs. The
     * chromosome only reorders members to shape the tree, so every individual is scored on the
     * same flows. Deriving the flows from the chromosome instead would let the GA score well by
     * pairing up neighbours rather than by routing better, and would produce an answer that
     * cannot be deployed - who talks to whom is an input from the applications, not an output
     * of the optimiser. Seng et al. likewise fix their transmissions by hand before routing.
     */
    std::vector<std::pair<std::uint32_t, std::uint32_t>> Pairs() const
    {
        std::vector<std::pair<std::uint32_t, std::uint32_t>> out;
        for (std::size_t i = 0; i + 1 < members.size(); i += 2)
        {
            out.emplace_back(members[i], members[i + 1]);
        }
        return out;
    }
};

} // namespace nsga

#endif // NSGA_SLICE_H
