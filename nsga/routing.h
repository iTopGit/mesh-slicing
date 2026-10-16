#ifndef NSGA_ROUTING_H
#define NSGA_ROUTING_H

#include "fitness.h"
#include "graph.h"
#include "individual.h"

#include "slice.h"

#include <array>
#include <cstdint>
#include <vector>

namespace nsga {

// One installable route: on node `node`, packets addressed to `dest` go to `nextHop`.
// Maps directly to ns-3 Ipv4StaticRouting::AddHostRouteTo(dest, nextHop, iface).
struct Route
{
    uint32_t node;
    uint32_t dest;
    uint32_t nextHop;
    int slice; // 0=eMBB, 1=URLLC, 2=mMTC (for reporting; routing itself is dest-only)
};

// Convert a chosen individual (the 3 slice orders) into a next-hop table: decode
// each slice's tree, then for every member (a destination) emit the next hop at
// every node on the unique tree path to it. Members are disjoint across slices, so
// no destination collides -> plain destination-based routing stays correct.
std::vector<Route> BuildRoutes(const Graph& g,
                               const Individual& ind,
                               const std::array<Slice, NUM_SLICES>& slices);

} // namespace nsga

#endif // NSGA_ROUTING_H
