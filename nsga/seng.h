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
 * Their total cost (their equation 9) is `a * sum of W over the slice's links + b * a node VNF
 * term`. This work places no VNFs, so the second term is zero and `a` scales a single sum, which
 * leaves the cost as the summed link weight. One number, minimised - so this is a weighted-sum
 * scalar method, and the comparison against it is exactly the "multi-objective beats a
 * pre-committed weighting" claim.
 */
double SengCost(const Individual& ind,
                const Graph& g,
                const std::array<Slice, NUM_SLICES>& slices);

/**
 * Minimise SengCost with the operators Seng describes: tournament selection of size 4, the same
 * order crossover and swap mutation this project already uses, and the same population size and
 * generation count as NSGA-II so the two methods get equal search effort. Returns the best
 * individual found.
 */
Individual SengRun(const Graph& g,
                   const std::array<Slice, NUM_SLICES>& slices,
                   const Params& p,
                   std::mt19937& rng);

} // namespace nsga

#endif // NSGA_SENG_H
