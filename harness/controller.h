#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../nsga/individual.h"

#include "ns3/net-device-container.h"
#include "ns3/queue-disc-container.h"

#include <array>
#include <cstdint>
#include <vector>

/**
 * Per-node admission controller: supplies the per-node ADMIT BUDGET to the allocator.
 *
 * VOCABULARY (settled 2026-10-15 with the user). Three words, one meaning each:
 *   capacity     - the real, measured ability to carry data (the warm-up link table). NOT here.
 *   admit budget - how much this node is WILLING to admit. A control variable, not a
 *                  measurement, and it can be wrong in either direction.
 *   limit        - the enforced ceiling the Tbf applies to one slice, derived from the budget.
 *
 * The budget is NOT capacity and must never be read as one. When the congestion signal does
 * not fire, the AIMD only ever adds `aiStep`, so the budget walks up until the clamp stops it:
 * measured at 10x10 / 5 Mb/s per slice it sits pinned near admitBudgetMax of 80 while the node
 * carries about 1 Mb/s, and at 9 Mb/s per slice it reads 49.7 against 5.45 actually offered.
 * Calling that "capacity" invited exactly the misreading CLAUDE.md rule 4 warns about.
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
        double urllcHeadroom = 0.0; // H = 1/(1 - D/T*), derived per topology; 0 = caller never set it
        double embbFloor = 0.3;      // F -- the design's one policy number
        double mmtcHeadroom = 1.2;   // declared so the lane drains (rho < 1); see harness/main.cc

        double aiStep = 1.0;   // additive increase per tick (Mb/s), TCP's +1
        double mdFactor = 0.5; // multiplicative decrease, TCP's x0.5
        uint32_t radioTargetPkts = 10; // above this the radio is behind, not just busy
        double period = 0.5;           // tick interval (s)

        // Pass 1 only. The two-round rule is kept as a measured arm, not as the default: it
        // reclaims on RECENT demand, so it takes exactly the slack that absorbs the NEXT burst.
        bool secondRound = false;

        double admitBudgetMin = 0.2;  // lower limit on the estimate, so a node can always recover
        double admitBudgetMax = 80.0; // upper limit on the estimate, not a measurement of capacity

        // FIXED RU-RATIO ARM. Each slice gets `fixedShare[s] * admitBudget`, demand never read --
        // the allocation Marrone et al. publish (one channel sliced by RU ratio, 0.1/0.3/0.6) and
        // the one Alloc.md 4c.1 rejects for this mesh, because a ratio reserves capacity against
        // traffic that may not exist. Kept as a measured COMPARATOR, never a default: it is what
        // the cited paper does, so the thesis has to answer it rather than ignore it.
        //
        // All zero = off, and the allocator runs normally. Order is eMBB, URLLC, mMTC.
        std::array<double, nsga::NUM_SLICES> fixedShare = {0.0, 0.0, 0.0};

        bool verbose = false;
    };

    /**
     * Seed every node at admitBudgetMax, then tick from `startDelay` seconds from now until
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
        return m_admitBudget;
    }

  private:
    void Tick();

    /** True when any fixed RU ratio was given, i.e. the Marrone-style comparator arm is on. */
    bool FixedShareOn() const
    {
        return m_cfg.fixedShare[0] > 0.0 || m_cfg.fixedShare[1] > 0.0 || m_cfg.fixedShare[2] > 0.0;
    }

    ns3::NetDeviceContainer m_devices;
    ns3::QueueDiscContainer m_qdiscs;
    Config m_cfg;
    double m_stop = 0.0;

    std::vector<double> m_admitBudget;
    std::vector<uint32_t> m_lastBacklog;
    std::vector<std::array<uint64_t, nsga::NUM_SLICES>> m_lastBytes;
};

#endif // CONTROLLER_H
