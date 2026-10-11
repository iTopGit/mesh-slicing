#include "applications.h"

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

array<uint64_t, NUM_SLICES>
PrintSliceResults(const array<Slice, NUM_SLICES>& slices,
                  Ptr<FlowMonitor> monitor,
                  Ptr<Ipv4FlowClassifier> classifier)
{
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
            tx += st.txPackets;
            rx += st.rxPackets;
            lost += st.lostPackets;
            rxBytes += st.rxBytes;
            rxPkts += st.rxPackets;
            delaySum += st.delaySum.GetSeconds();
            if (st.rxPackets > 0)
            {
                firstRx = min(firstRx, st.timeFirstRxPacket.GetSeconds());
                lastRx = max(lastRx, st.timeLastRxPacket.GetSeconds());
            }
        }
        double dur = (lastRx > firstRx) ? (lastRx - firstRx) : 0.0;
        double thpt = (dur > 0) ? (rxBytes * 8.0 / dur / 1e6) : 0.0;
        double delay = (rxPkts > 0) ? (delaySum / rxPkts * 1e3) : 0.0;
        // FlowMonitor's own lostPackets, NOT tx-rx. The two are different and tx-rx is not loss:
        // measured on 5x5, tx-rx was 183/271/86 across the slices while lostPackets was 0/35/0 -
        // and 35 is exactly what our URLLC bucket dropped. The rest are packets whose delivery
        // FlowMonitor never resolved; 18 s of extra drain does not change the figure and no drop
        // trace (MAC, IP, ARP) and no queue accounts for them.
        double loss = (tx > 0) ? (100.0 * lost / tx) : 0.0;
        srcPackets[si] = tx;
        cout << "slice " << s.name << " (" << s.members.size() << " members, " << nPairs
             << " pairs, " << nFlows << " flows): tx=" << tx << " rx=" << rx << " thpt=" << thpt << "Mb/s delay=" << delay
             << "ms loss=" << loss << "%\n";
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
