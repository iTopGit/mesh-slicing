// The mesh as a weighted graph: nodes + undirected links, each link carrying a
// measured capacity (R) and base delay. Built either from a real warm-up CSV or a
// synthetic grid, and it provides the shortest-path search the decoder relies on.
#include "graph.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>

using namespace std;

namespace nsga {

pair<uint32_t, uint32_t>
Graph::Key(uint32_t a, uint32_t b)
{
    return (a < b) ? make_pair(a, b) : make_pair(b, a);
}

void
Graph::AddLink(uint32_t a, uint32_t b, const Link& l)
{
    // emplace fails on a duplicate edge -> extend adjacency only for a genuinely new link
    if (mLinks.emplace(Key(a, b), l).second)
    {
        mAdj[a].push_back(b);
        mAdj[b].push_back(a);
    }
}

Graph
Graph::FromGrid(uint32_t width, uint32_t height, double baseDelayMs, double capacityMbps)
{
    Graph g;
    g.mNum = width * height;
    g.mAdj.resize(g.mNum);
    Link l{baseDelayMs, capacityMbps};
    for (uint32_t r = 0; r < height; ++r)
    {
        for (uint32_t c = 0; c < width; ++c)
        {
            uint32_t id = r * width + c;
            if (c + 1 < width)
            {
                g.AddLink(id, id + 1, l); // right neighbor
            }
            if (r + 1 < height)
            {
                g.AddLink(id, id + width, l); // down neighbor
            }
        }
    }
    return g;
}

Graph
Graph::FromLinkTable(const string& csvPath, uint32_t numNodes)
{
    ifstream in(csvPath);
    if (!in)
    {
        throw runtime_error("cannot open link table: " + csvPath);
    }
    Graph g;
    g.mNum = numNodes;
    g.mAdj.resize(numNodes);
    string line;
    getline(in, line); // header
    while (getline(in, line))
    {
        if (line.empty())
        {
            continue;
        }
        stringstream ss(line);
        string tok;
        uint32_t a, b;
        double delayMs, capBps;
        getline(ss, tok, ',');
        a = stoul(tok);
        getline(ss, tok, ',');
        b = stoul(tok);
        getline(ss, tok, ',');
        delayMs = stod(tok);
        getline(ss, tok, ',');
        capBps = stod(tok);
        g.AddLink(a, b, Link{delayMs, capBps / 1e6});
    }
    return g;
}

bool
Graph::HasLink(uint32_t a, uint32_t b) const
{
    return mLinks.count(Key(a, b)) > 0;
}

const Link&
Graph::GetLink(uint32_t a, uint32_t b) const
{
    return mLinks.at(Key(a, b));
}

// Dijkstra shortest path from `src` (weighted by base delay), but with an early
// exit: the first node it settles that's already in the tree (`inSet`) is the
// closest one, so it stops there and walks prev[] back to rebuild the path.
vector<uint32_t>
Graph::NearestInSet(uint32_t src, const vector<char>& inSet) const
{
    if (inSet[src]) // src already in the tree -> zero-length path
    {
        return {src};
    }
    const double INF = numeric_limits<double>::infinity();
    vector<double> dist(mNum, INF);      // best known distance to each node
    vector<uint32_t> prev(mNum, mNum);   // predecessor for path rebuild; mNum = "none"
    dist[src] = 0.0;

    // Min-heap keyed on distance: always expand the closest unsettled node next.
    using QItem = pair<double, uint32_t>; // (dist, node)
    priority_queue<QItem, vector<QItem>, greater<QItem>> pq;
    pq.push({0.0, src});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) // stale entry: a shorter dist was set after this was queued
        {
            continue;
        }
        if (inSet[u] && u != src) // nearest set node popped -> done
        {
            // rebuild by following prev[] from the target back to src, then flip it
            vector<uint32_t> path;
            for (uint32_t x = u; x != mNum; x = prev[x])
            {
                path.push_back(x);
            }
            reverse(path.begin(), path.end());
            return path;
        }
        for (uint32_t v : mAdj[u])
        {
            double w = GetLink(u, v).baseDelayMs;
            if (dist[u] + w < dist[v])
            {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
    return {}; // unreachable
}

} // namespace nsga
