#ifndef NSGA_SENG_H
#define NSGA_SENG_H

#include "graph.h"
#include "individual.h"
#include "nsga2.h"
#include "slice.h"

#include <array>
#include <random>

namespace nsga {

/**
 * Seng et al.'s single-objective routing cost, ported for use as the scalar baseline.
 *
 * Their link weight (their equation 3) grows exponentially with how loaded a link is and goes
 * infinite once it is over capacity:
 *
 *     B(e) = Bmax / Nch                                  capacity after channel reuse
 *     W(e) = exp(7 * B_used * Nch / Bmax)   if within    the 7 is theirs, chosen empirically so
 *          = infinity                       if over      W = 1000 when B_used * Nch = Bmax
 *
 * `Nch` is "the number of identical channels used in the reception range of a node" - in a
 * single-channel mesh that is the size of the link's interference neighbourhood, which is the
 * same set `Evaluate` already builds from `Graph::Hears`. Their paper states the assumption
 * directly: "the maximum possible bit rate on the channel is divided by the number of devices
 * that operate on this channel", which is independent support for our own contention domain.
 *
 * WHAT THIS IMPLEMENTS, AND WHAT IT DOES NOT. Corrected 2026-10-12 after reading the paper in
 * full; the previous note here was wrong on both terms of their equation 9.
 *
 * Their equation 9, in their notation:
 *
 *     cost = alpha * SUM(e in E_s)  +  SUM(n in V_t U V_f) beta * [ SUM(e in E_n) W(e)
 *                                                                   / SUM(e in E_n) ]
 *
 * and the two criteria they state for it are (1) "the length of the paths of a slice", i.e. a
 * LINK COUNT, and (2) "the average utilization of a node the path contains which is no AP node".
 *
 * So the second term is NOT a VNF-placement term - it runs over `V_t U V_f`, transport nodes
 * included, and in an all-relay mesh like ours it does not vanish. And the first term counts
 * links; it is not the sum of W. (The earlier claim here, that the term is zero "because this
 * work places no VNFs", was a misreading of both.)
 *
 * **Equation 9 is not implementable as published.** `alpha` and `beta` are described only as
 * "adjustable parameters used to modify optimization weighting"; no values appear anywhere in the
 * paper, and the Appendix nomenclature (their Table 13) has no entry for either. Picking them
 * ourselves would invent the two numbers that decide how their method trades path length against
 * node load - the whole content of their weighting.
 *
 * What is implemented instead is their **equation 3 weight summed over the slice's links**, with
 * their over-capacity rule (infinite once B * Nch > Bmax). That is one scalar, minimised, built
 * from their published link model with no invented constant - which is what the comparison needs:
 * a pre-committed single weighting against NSGA-II's front. It is not a reproduction of their
 * equation 9, and no result from it should be described as reproducing their published numbers.
 *
 * Their algorithm has three further components this port does not carry, all from their Section
 * VI: rank selection used with equal probability alongside tournament (their Algorithm 2.1), a
 * crossover rate adapted dynamically (their Algorithm 2.2), and a **2-opt local search applied
 * after mutation** (their Algorithm 2.3), which makes their EA memetic rather than plain. Their
 * reported runs use population 20 and 10 generations. This port matches NSGA-II's population and
 * generation count instead, so neither method gets more search effort - a deliberate fairness
 * choice that is also a deviation, and both facts belong in any write-up of the comparison.
 */
double SengCost(const Individual& ind,
                const Graph& g,
                const std::array<Slice, NUM_SLICES>& slices);

/**
 * Minimise SengCost with the operators Seng describes: tournament selection of size 4 (theirs -
 * "Four randomly selected elements are evaluated using the function, and the best one is chosen",
 * their Algorithm 1.1), the same order crossover and swap mutation this project already uses, and
 * the same population size and generation count as NSGA-II so the two methods get equal search
 * effort. Their Section VI improvements are not ported; see the note on SengCost. Returns the best
 * individual found.
 */
Individual SengRun(const Graph& g,
                   const std::array<Slice, NUM_SLICES>& slices,
                   const Params& p,
                   std::mt19937& rng);

} // namespace nsga

#endif // NSGA_SENG_H
