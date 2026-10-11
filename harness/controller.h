#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../nsga/individual.h"

#include "ns3/net-device-container.h"
#include "ns3/queue-disc-container.h"

#include <array>
#include <cstdint>
#include <vector>

/**
 * Per-node capacity controller: the thing that supplies `C` to the allocator.
 *
 * EVERY NODE RUNS ITS OWN LOOP ON ITS OWN OBSERVATIONS. Nothing is exchanged between
 * nodes, there is no coordinator, and no node reads another node's state. That is the
 * point: a mesh has no centre, and a design that needs one gives away the property the
 * thesis is about. The precedent is TCP, whose AIMD constants these are (+1 / x0.5) and
 * which every endpoint runs on purely local evidence.
 *
 * It is also more accurate than a single figure. Capacity is not global: a 5x5 grid spans
 * 200 m, the centre node hears eight neighbours and a corner node three, and they really do
 * have different capacity. One number would force the worst corner to govern a quiet one.
 *
 * END TO END STILL HOLDS. A flow crossing several nodes meets each node's buckets in turn,
 * so it ends up limited by the TIGHTEST node on its path -- which is the end-to-end
 * bottleneck, discovered without anyone computing it. Caps do not compound: a looser
 * downstream cap never bites on an already-capped stream.
 *
 * The cost of having no coordinator, and it should be measured rather than hidden: when a
 * downstream node is tighter than an upstream one, the upstream node has already spent
 * airtime on packets the downstream node will drop.
 *
 * Both inputs are local:
 *   capacity  AIMD on this radio's own WifiMacQueue depth (AC_BE -- every slice marks
 *             AC_BE, so one MAC queue carries all of it). Four queue-disc-level sensors
 *             were tried first and all failed; see the capacity-feedback-gate header.
 *   demand    bytes offered to this node's own buckets since the last tick, counted
 *             BEFORE policing. A node that relays no mMTC traffic reads mMTC demand 0 and
 *             reserves it nothing, which a configured figure could not express.
 */
class SliceController
{
  public:
    struct Config
    {
        double urllcHeadroom = 1.64; // 1/(1 - T0/T*); Alloc.md 0.6
        double embbFloor = 0.3;      // F -- the design's one policy number
        double mmtcHeadroom = 1.0;   // constant-rate source needs none; Alloc.md 4c.5

        double aiStep = 1.0;   // additive increase per tick (Mb/s), TCP's +1
        double mdFactor = 0.5; // multiplicative decrease, TCP's x0.5
        uint32_t radioTargetPkts = 10; // above this the radio is behind, not just busy
        double period = 0.5;           // tick interval (s)

        double capMin = 0.2;   // floor, so a node can always recover
        double capMax = 80.0;  // physical ceiling, NOT a capacity estimate
        bool verbose = false;
    };

    /**
     * Seed every node at capMax, then tick from `startDelay` seconds from now until
     * `duration` seconds after that. Both are RELATIVE: the harness reaches here with the
     * sim clock already advanced through the warm-up sweep (~190 s on a 5x5), so absolute
     * times would put the whole traffic phase in the past.
     *
     * `seedDemand` is used only for the initial apply, before any traffic has arrived to
     * measure; every tick after that reads the node's own buckets.
     */
    void Start(const ns3::NetDeviceContainer& devices,
               const ns3::QueueDiscContainer& qdiscs,
               const std::array<double, nsga::NUM_SLICES>& seedDemand,
               const Config& cfg,
               double startDelay,
               double duration);

    /** Per-node capacities at the end of the run, for reporting. */
    const std::vector<double>& Capacities() const
    {
        return m_cap;
    }

  private:
    void Tick();

    ns3::NetDeviceContainer m_devices;
    ns3::QueueDiscContainer m_qdiscs;
    Config m_cfg;
    double m_stop = 0.0;

    std::vector<double> m_cap;
    std::vector<uint32_t> m_lastBacklog;
    std::vector<std::array<uint64_t, nsga::NUM_SLICES>> m_lastBytes;
};

#endif // CONTROLLER_H
