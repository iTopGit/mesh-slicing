#ifndef NSGA_NSGA2_H
#define NSGA_NSGA2_H

#include "fitness.h"
#include "graph.h"
#include "individual.h"

#include <array>
#include <random>
#include <vector>

namespace nsga {

struct Params
{
    uint32_t popSize = 100;
    uint32_t generations = 100;
    double mutationRate = 0.1; // per-segment swap-mutation probability
    double C = 4.53;           // calibrated delay constant (PLAN.md §10)
    bool trace = false;        // print per-generation best-objective progress
    int showGen = -1;          // dump one individual at this generation (-1 = never)
};

// a dominates b iff no worse on every objective and strictly better on one
// (each objective read in its own sense, OBJ_MAXIMIZE).

// Fast non-dominated sort: sets each individual's rank, returns fronts as index
// lists (front 0 = Pareto-best).

// Crowding distance for one front, written into pop[i].crowd.

// Run NSGA-II; returns front 0 (Pareto set) of the final population.
/**
 * The GA's variation operators, shared so that every optimiser in this project searches with the
 * SAME ones. The Seng baseline had its own copies and they were not equivalent - its crossover
 * filled from index 0 while this one fills cyclically from the cut - which would have let a win in
 * the comparison come from the operator rather than from the cost function being compared.
 */
std::vector<std::uint32_t> OrderCrossover(const std::vector<std::uint32_t>& p1,
                                          const std::vector<std::uint32_t>& p2,
                                          std::mt19937& rng);
void SwapMutation(std::vector<std::uint32_t>& seg, double rate, std::mt19937& rng);

std::vector<Individual> Run(const Graph& g,
                            const std::array<Slice, NUM_SLICES>& slices,
                            const Params& p,
                            std::mt19937& rng);

// Knee point of a Pareto front (final joint pick, PLAN.md §6): normalize each
// objective to [0,1] with 0 = best, return the index of the solution closest to
// the ideal point — the balanced best-compromise, no pre-set weights.
size_t KneePoint(const std::vector<Individual>& front);

} // namespace nsga

#endif // NSGA_NSGA2_H
