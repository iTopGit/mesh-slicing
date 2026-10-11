// Per-node AIMD capacity controller. Design and rationale: controller.h, Alloc.md 4c.
#include "controller.h"

#include "traffic-control.h"

#include "../ofdma/allocator.h"

#include "ns3/simulator.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-net-device.h"

#include <algorithm>
#include <iomanip>
#include <iostream>

using namespace ns3;
using namespace nsga;
using namespace std;

namespace {

// This radio's own transmit backlog. Read the device, not the queue disc: the buckets admit
// protected traffic immediately, so L2.5 queues are found EMPTY at every sample even while
// end-to-end delay is hundreds of ms. The waiting happens below, in the MAC.
uint32_t
RadioBacklog(Ptr<NetDevice> device)
{
    Ptr<WifiNetDevice> wifi = DynamicCast<WifiNetDevice>(device);
    if (!wifi)
    {
        return 0;
    }
    Ptr<WifiMacQueue> q = wifi->GetMac()->GetTxopQueue(AC_BE);
    return q ? q->GetNPackets() : 0;
}

} // namespace

void
SliceController::Start(const NetDeviceContainer& devices,
                       const QueueDiscContainer& qdiscs,
                       const array<double, NUM_SLICES>& seedDemand,
                       const Config& cfg,
                       double startDelay,
                       double duration)
{
    m_devices = devices;
    m_qdiscs = qdiscs;
    m_cfg = cfg;
    // Relative to NOW: the harness arrives here with the clock already past the warm-up sweep.
    m_stop = Simulator::Now().GetSeconds() + startDelay + duration;

    const uint32_t n = qdiscs.GetN();
    m_cap.assign(n, cfg.capMax);
    m_lastBacklog.assign(n, 0);
    m_lastBytes.assign(n, {});

    // Seed from the configured demands: no traffic has arrived yet, so there is nothing to
    // measure. Starting at capMax also means every cap begins ABOVE demand, so nobody is
    // policed until the medium actually fills -- a cap that bites with capacity to spare is
    // the failure mode this whole design exists to avoid.
    for (uint32_t i = 0; i < n; ++i)
    {
        SetSliceRatesOn(qdiscs,
                        i,
                        ofdma::Allocate(m_cap[i],
                                        seedDemand,
                                        cfg.urllcHeadroom,
                                        cfg.embbFloor,
                                        cfg.mmtcHeadroom));
    }

    Simulator::Schedule(Seconds(startDelay), &SliceController::Tick, this);
}

void
SliceController::Tick()
{
    const double now = Simulator::Now().GetSeconds();
    uint32_t congested = 0;

    for (uint32_t i = 0; i < m_qdiscs.GetN(); ++i)
    {
        const uint32_t backlog = RadioBacklog(m_devices.Get(i));
        const bool over = backlog > m_cfg.radioTargetPkts;
        const bool draining = backlog < m_lastBacklog[i];

        // Three states. A queue takes time to drain, so cutting again while the previous cut
        // is still working over-corrects. A falling backlog is itself the evidence that the
        // last cut was enough, so HOLD needs no extra constant.
        if (over && !draining)
        {
            m_cap[i] *= m_cfg.mdFactor;
            ++congested;
        }
        else if (!over)
        {
            m_cap[i] += m_cfg.aiStep;
        }
        m_lastBacklog[i] = backlog;
        m_cap[i] = clamp(m_cap[i], m_cfg.capMin, m_cfg.capMax);

        // Local demand: bytes offered to this node's buckets since the last tick. Offered,
        // not admitted -- see SliceOfferedBytes.
        array<uint64_t, NUM_SLICES> bytes = SliceOfferedBytes(m_qdiscs, i);
        array<double, NUM_SLICES> demand{};
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            demand[s] = (bytes[s] - m_lastBytes[i][s]) * 8.0 / m_cfg.period / 1e6;
        }
        m_lastBytes[i] = bytes;

        SetSliceRatesOn(m_qdiscs,
                        i,
                        ofdma::Allocate(m_cap[i],
                                        demand,
                                        m_cfg.urllcHeadroom,
                                        m_cfg.embbFloor,
                                        m_cfg.mmtcHeadroom));
    }

    if (m_cfg.verbose)
    {
        double sum = 0.0;
        double lo = m_cap.empty() ? 0.0 : *min_element(m_cap.begin(), m_cap.end());
        double hi = m_cap.empty() ? 0.0 : *max_element(m_cap.begin(), m_cap.end());
        for (double c : m_cap)
        {
            sum += c;
        }
        cout << "  t=" << fixed << setprecision(1) << now << "s  congested " << congested << "/"
             << m_qdiscs.GetN() << " nodes" << setprecision(2) << "   C min/mean/max = " << lo
             << "/" << (m_cap.empty() ? 0.0 : sum / m_cap.size()) << "/" << hi << " Mb/s\n";
    }

    if (now + m_cfg.period < m_stop)
    {
        Simulator::Schedule(Seconds(m_cfg.period), &SliceController::Tick, this);
    }
}
