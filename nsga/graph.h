#ifndef NSGA_GRAPH_H
#define NSGA_GRAPH_H

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace nsga {

// One mesh link's measured properties (from the warm-up link table).
struct Link
{
    double baseDelayMs = 0.0;
    double capacityMbps = 0.0;
};

// Undirected weighted mesh: grid adjacency + per-link R/delay. The decoder walks
// it (shortest path by base delay); fitness reads per-link capacity for contention.
class Graph
{
  public:
    // Load links from a warm-up CSV `a,b,base_delay_ms,capacity_bps`.
    static Graph FromLinkTable(const std::string& csvPath, uint32_t numNodes);
    // Synthesize an orthogonal grid with uniform link values (self-contained runs).
    static Graph FromGrid(uint32_t width, uint32_t height, double baseDelayMs, double capacityMbps);

    uint32_t NumNodes() const { return mNum; }
    bool HasLink(uint32_t a, uint32_t b) const;
    const Link& GetLink(uint32_t a, uint32_t b) const;

    // Dijkstra from `src` (base-delay cost) to the nearest node flagged in `inSet`.
    // Returns the path src..target (inclusive); {src} if src is already in the set;
    // empty if none reachable.
    std::vector<uint32_t> NearestInSet(uint32_t src, const std::vector<char>& inSet) const;

  private:
    static std::pair<uint32_t, uint32_t> Key(uint32_t a, uint32_t b);
    void AddLink(uint32_t a, uint32_t b, const Link& l);

    uint32_t mNum = 0;
    std::vector<std::vector<uint32_t>> mAdj;
    std::map<std::pair<uint32_t, uint32_t>, Link> mLinks;
};

} // namespace nsga

#endif // NSGA_GRAPH_H
