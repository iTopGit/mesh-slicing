#include "routing.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <map>
#include <queue>

using namespace std;

namespace nsga {

vector<Route>
BuildRoutes(const Graph& g, const Individual& ind, const array<Slice, NUM_SLICES>& slices)
{
    vector<Route> routes;
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        Tree t = DecodeSlice(g, ind.seg[s]);

        // Adjacency over just this slice's tree edges (undirected).
        map<uint32_t, vector<uint32_t>> adj;
        for (const EdgeKey& e : t.links)
        {
            adj[e.first].push_back(e.second);
            adj[e.second].push_back(e.first);
        }

        // For each member as destination, BFS the tree outward. When node v is first
        // reached from u, u is one hop closer to dest -> v's next hop toward dest is u.
        for (uint32_t dest : slices[s].members)
        {
            map<uint32_t, uint32_t> seen; // node -> next hop toward dest
            queue<uint32_t> q;
            seen[dest] = dest; // visited marker for the destination itself
            q.push(dest);
            while (!q.empty())
            {
                uint32_t u = q.front();
                q.pop();
                for (uint32_t v : adj[u])
                {
                    if (seen.count(v))
                    {
                        continue;
                    }
                    seen[v] = u;
                    routes.push_back({v, dest, u, s});
                    q.push(v);
                }
            }
        }
    }
    return routes;
}

} // namespace nsga
