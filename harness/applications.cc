#include "applications.h"

#include "loss-ledger.h" // HeldPackets: the settle tripwire reads the same counter as the drain

#include "ns3/core-module.h"
#include "ns3/data-rate.h"
#include "ns3/flow-monitor.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"

#include <algorithm>
#include <iostream>

using namespace ns3;
using namespace nsga;
using namespace std;

namespace {

// One ToS-marked constant-rate UDP flow src->dst on `port`, plus its sink at dst.
void
AddFlow(const MeshTopology& mesh,
        const Ipv4InterfaceContainer& interfaces,
        uint32_t src,
        uint32_t dst,
        uint16_t port,
        uint64_t rateBps,
        uint8_t tos,
        double startSec,
        double stopSec,
        uint32_t pktSize)
{
    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(interfaces.GetAddress(dst), port));
    onoff.SetConstantRate(DataRate(rateBps), pktSize);
    onoff.SetAttribute("Tos", UintegerValue(tos)); // slice mark (SliceTos)
    ApplicationContainer s = onoff.Install(mesh.nodes.Get(src));
    s.Start(Seconds(startSec));
    s.Stop(Seconds(stopSec));

    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer r = sink.Install(mesh.nodes.Get(dst));
    r.Start(Seconds(startSec));
    r.Stop(Seconds(stopSec + 1.0));
}

} // namespace

void
InstallSliceTraffic(const MeshTopology& mesh,
                    const Ipv4InterfaceContainer& interfaces,
                    const Slice& slice,
                    double onWireMbps,
                    bool bidirectional,
                    double startSec,
                    double stopSec,
                    uint32_t pktSize)
{
    auto pairs = slice.Pairs();
    if (pairs.empty())
    {
        return;
    }
    uint32_t numFlows = (bidirectional ? 2 : 1) * static_cast<uint32_t>(pairs.size());
    uint64_t perFlowBps = static_cast<uint64_t>(onWireMbps * 1e6 / numFlows);

    // Stride 2 per pair whether or not the reverse flow exists, so the slice's port span is
    // always [port, port + 2*pairs) and PrintSliceResults' range does not shift between arms.
    uint16_t port = slice.port;
    for (const auto& [src, dst] : pairs)
    {
        AddFlow(mesh, interfaces, src, dst, port, perFlowBps, slice.tos, startSec, stopSec,
                pktSize);
        if (bidirectional)
        {
            AddFlow(mesh, interfaces, dst, src, port + 1, perFlowBps, slice.tos, startSec,
                    stopSec, pktSize);
        }
        port += 2;
    }
}

void
OpenFlowWindow(const MeshTopology& mesh,
               Ptr<FlowMonitor> monitor,
               FlowWindow& w,
               const string& note)
{
    w.base.clear();
    for (const auto& [id, st] : monitor->GetFlowStats())
    {
        w.base[id] = {st.txPackets, st.rxPackets, st.rxBytes, st.delaySum.GetSeconds()};
    }
    w.openSec = Simulator::Now().GetSeconds();
    w.note = note;
    w.heldOpen = HeldPackets(mesh, nullptr, nullptr);
    cout << "measurement window OPEN at t=" << w.openSec << "s over " << w.base.size()
         << " flow(s); held=" << w.heldOpen << (note.empty() ? "" : "; ") << note << "\n";
}

void
CloseFlowWindow(const MeshTopology& mesh, FlowWindow& w, double growthFactor)
{
    w.heldClose = HeldPackets(mesh, nullptr, nullptr);
    w.closed = true;
    const bool grew = w.heldOpen > 0 && w.heldClose > growthFactor * w.heldOpen;
    cout << "  window [" << w.openSec << "s, " << Simulator::Now().GetSeconds()
         << "s]: held " << w.heldOpen << " -> " << w.heldClose
         << (grew ? "  NOT SETTLED (grew past x" : "  settled (within x") << growthFactor
         << ")\n";
}

bool
WindowsSettled(const vector<FlowWindow>& windows, double growthFactor)
{
    for (const FlowWindow& w : windows)
    {
        if (w.closed && w.heldOpen > 0 && w.heldClose > growthFactor * w.heldOpen)
        {
            return false;
        }
    }
    return true;
}

array<uint64_t, NUM_SLICES>
PrintSliceResults(const array<Slice, NUM_SLICES>& slices,
                  Ptr<FlowMonitor> monitor,
                  Ptr<Ipv4FlowClassifier> classifier,
                  double trafficSec,
                  const FlowWindow* window)
{
    // Force FlowMonitor to settle its accounts before anything is read. `--maxPerHopDelay` is set
    // high (120 s) ON PURPOSE during the run, so a slow packet is not written off as lost while it
    // is still travelling - but that same setting means a packet killed at the MAC is still
    // "in flight" when the run ends, and reads as delivered-nowhere rather than lost. The drain has
    // already stopped, by emptiness or by a stalled queue, so nothing can still arrive: every
    // packet not yet received is lost, and 0 s is the honest threshold at this point.
    //
    // Without this the ledger and FlowMonitor contradict each other outright - a run with 1400 MAC
    // lifetime expiries and rx = 4226 of tx = 5710 printed loss = 0.000%.
    monitor->CheckForLostPackets(Seconds(0));

    array<uint64_t, NUM_SLICES> srcPackets = {};
    cout << "\n=== per-slice results ===\n";
    for (int si = 0; si < NUM_SLICES; ++si)
    {
        const Slice& s = slices[si];
        size_t nPairs = s.members.size() / 2;
        if (nPairs == 0)
        {
            continue;
        }
        uint16_t lo = s.port;
        uint16_t hi = static_cast<uint16_t>(s.port + 2 * nPairs); // one-past its last flow port
        uint64_t tx = 0, rx = 0, rxBytes = 0, rxPkts = 0, lost = 0;
        // Whole-run totals, kept alongside the windowed ones. `lost` and the queue-disc counters
        // are both whole-run, so they must be divided by whole-run tx or the ratio mixes scopes.
        uint64_t txAll = 0;
        int nFlows = 0; // counted, not assumed: the arm may be one-way or two-way
        // FlowMonitor already attributes every drop to a reason; read it rather than guessing
        // which trace source to hook.
        array<uint64_t, 9> dropped = {};
        double delaySum = 0.0, firstRx = 1e18, lastRx = 0.0;
        for (const auto& [id, st] : monitor->GetFlowStats())
        {
            uint16_t p = classifier->FindFlow(id).destinationPort;
            if (p < lo || p >= hi)
            {
                continue;
            }
            ++nFlows;
            for (size_t r = 0; r < dropped.size() && r < st.packetsDropped.size(); ++r)
            {
                dropped[r] += st.packetsDropped[r];
            }
            // Inside a measurement window, subtract the counters captured when it opened. Only
            // the ACCUMULATING fields can be differenced; `lostPackets` cannot, because
            // CheckForLostPackets attributes loss when the run ENDS rather than when it happens,
            // so a differenced loss figure would be an artefact of when the sweep ran. Loss is
            // therefore a whole-run figure and is labelled as one below.
            uint64_t fTx = st.txPackets, fRx = st.rxPackets, fBytes = st.rxBytes;
            double fDelay = st.delaySum.GetSeconds();
            if (window)
            {
                auto it = window->base.find(id);
                if (it != window->base.end())
                {
                    fTx -= min(fTx, it->second.txPackets);
                    fRx -= min(fRx, it->second.rxPackets);
                    fBytes -= min(fBytes, it->second.rxBytes);
                    fDelay = max(0.0, fDelay - it->second.delaySum);
                }
            }
            txAll += st.txPackets;
            tx += fTx;
            rx += fRx;
            lost += st.lostPackets;
            rxBytes += fBytes;
            rxPkts += fRx;
            delaySum += fDelay;
            if (st.rxPackets > 0)
            {
                firstRx = min(firstRx, st.timeFirstRxPacket.GetSeconds());
                lastRx = max(lastRx, st.timeLastRxPacket.GetSeconds());
            }
        }
        // Goodput is divided by the OFFERED-TRAFFIC window, never by the arrival span. The span
        // ends with the last packet out of the drain, so it stretches with whatever backlog the
        // run happened to build: two arms that sent the same 5710 packets and lost none printed
        // 3.666 against 3.976 Mb/s purely because one drained in 1.2 s and the other in 0.4 s -
        // same work, different stopwatch, and it was once reported as a difference. Bytes that
        // arrive during the drain were still offered inside the window, so they count here.
        double span = (lastRx > firstRx) ? (lastRx - firstRx) : 0.0;
        double thpt = (trafficSec > 0) ? (rxBytes * 8.0 / trafficSec / 1e6) : 0.0;
        double spanThpt = (span > 0) ? (rxBytes * 8.0 / span / 1e6) : 0.0;
        double delay = (rxPkts > 0) ? (delaySum / rxPkts * 1e3) : 0.0;
        // FlowMonitor's own lostPackets, NOT tx-rx. The two are different and tx-rx is not loss:
        // measured on 5x5, tx-rx was 183/271/86 across the slices while lostPackets was 0/35/0 -
        // and 35 is exactly what our URLLC bucket dropped. The rest are packets whose delivery
        // FlowMonitor never resolved; 18 s of extra drain does not change the figure and no drop
        // trace (MAC, IP, ARP) and no queue accounts for them.
        // Loss is WHOLE-RUN, numerator and denominator both: see the note in the flow loop on why
        // lostPackets cannot be windowed. Printed with its scope named so it is never read as a
        // window figure beside the windowed throughput and delay.
        double loss = (txAll > 0) ? (100.0 * lost / txAll) : 0.0;
        srcPackets[si] = txAll;
        cout << "slice " << s.name << " (" << s.members.size() << " members, " << nPairs
             << " pairs, " << nFlows << " flows): tx=" << tx << " rx=" << rx << " thpt=" << thpt << "Mb/s delay=" << delay
             << "ms loss=" << loss << "%" << (window ? " (thpt+delay over the window; loss whole-run)" : "")
             << "\n";
        // Printed as a cross-check, not as a result: a large gap means a long drain tail, which is
        // exactly the artefact the window divisor removes.
        cout << "    over the " << span << "s arrival span it would read " << spanThpt
             << "Mb/s (cross-check)\n";
        static const array<const char*, 9> why = {"no route", "ttl expired", "bad checksum",
                                                  "queue", "queue disc", "interface down",
                                                  "route error", "fragment timeout", "unknown"};
        uint64_t accounted = 0;
        for (size_t r = 0; r < dropped.size(); ++r)
        {
            accounted += dropped[r];
            if (dropped[r])
            {
                cout << "    dropped, " << why[r] << ": " << dropped[r] << "\n";
            }
        }
        if (tx > rx + lost)
        {
            cout << "    unresolved by FlowMonitor (not loss): " << (tx - rx - lost) << "\n";
        }
    }
    return srcPackets;
}
