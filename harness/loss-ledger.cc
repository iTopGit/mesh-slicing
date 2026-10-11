#include "loss-ledger.h"

#include "ns3/arp-cache.h"
#include "ns3/config.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/queue-disc.h"
#include "ns3/traffic-control-layer.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"

#include <array>
#include <iostream>

using namespace ns3;
using namespace std;

namespace {

array<uint64_t, 4> g_macDrops = {0, 0, 0, 0}; // indexed by WifiMacDropReason
uint64_t g_ipDrops = 0;
uint64_t g_arpDrops = 0;
uint64_t g_arpQueueDrops = 0;
array<uint64_t, 16> g_ipReason = {};

void
OnMacDrop(WifiMacDropReason reason, Ptr<const WifiMpdu>)
{
    size_t i = static_cast<size_t>(reason);
    if (i < g_macDrops.size())
    {
        ++g_macDrops[i];
    }
}

void
OnArpDrop(Ptr<const Packet>)
{
    ++g_arpDrops;
}

void
OnArpQueueDrop(Ptr<const Packet>)
{
    ++g_arpQueueDrops;
}

void
OnIpDrop(const Ipv4Header&, Ptr<const Packet>, Ipv4L3Protocol::DropReason reason, Ptr<Ipv4>, uint32_t)
{
    ++g_ipDrops;
    size_t i = static_cast<size_t>(reason);
    if (i < g_ipReason.size())
    {
        ++g_ipReason[i];
    }
}

} // namespace

void
InstallLossLedger(const MeshTopology& mesh)
{
    uint32_t macOk = 0, ipOk = 0;
    for (uint32_t i = 0; i < mesh.devices.GetN(); ++i)
    {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(mesh.devices.Get(i));
        if (dev && dev->GetMac() &&
            dev->GetMac()->TraceConnectWithoutContext("DroppedMpdu", MakeCallback(&OnMacDrop)))
        {
            ++macOk;
        }
    }
    for (uint32_t i = 0; i < mesh.nodes.GetN(); ++i)
    {
        Ptr<Ipv4L3Protocol> ip = mesh.nodes.Get(i)->GetObject<Ipv4L3Protocol>();
        if (ip && ip->TraceConnectWithoutContext("Drop", MakeCallback(&OnIpDrop)))
        {
            ++ipOk;
        }
    }
    // A silent failure here reads as "the medium lost nothing", which is the one answer that
    // must never be guessed.
    // ArpCache's Drop fires when a packet waiting on an ARP reply is pushed out: the pending
    // queue holds only 3 packets (arp-cache.cc) and entries go stale after 120 s, so a flow that
    // starts after a long quiet gap loses its opening burst with no other counter moving.
    Config::ConnectWithoutContext("/NodeList/*/$ns3::ArpL3Protocol/CacheList/*/Drop",
                                  MakeCallback(&OnArpDrop));
    // ArpL3Protocol has its OWN Drop, a different trace source from the cache's, and it is the
    // one that fires when the pending queue is already full (arp-l3-protocol.cc:320). Nothing
    // else records it: ipv4-interface.cc:295 drops a packet whose Lookup returns false with no
    // trace at all, so an unresolvable next hop is silent in every other counter. Without this
    // hook a flow can lose 100% of its packets and every figure still reads zero.
    Config::ConnectWithoutContext("/NodeList/*/$ns3::ArpL3Protocol/Drop",
                                  MakeCallback(&OnArpQueueDrop));
    cout << "loss ledger: MAC traces " << macOk << "/" << mesh.devices.GetN() << ", IP traces "
         << ipOk << "/" << mesh.nodes.GetN() << endl;
}

static uint32_t
CountDisc(Ptr<QueueDisc> d)
{
    uint32_t n = d->GetNPackets();
    for (uint32_t i = 0; i < d->GetNQueueDiscClasses(); ++i)
    {
        n += CountDisc(d->GetQueueDiscClass(i)->GetQueueDisc());
    }
    return n;
}

uint32_t
HeldPackets(const MeshTopology& mesh, uint32_t* inQdiscs, uint32_t* inRadios)
{
    uint32_t qdisc = 0, mac = 0;
    for (uint32_t i = 0; i < mesh.devices.GetN(); ++i)
    {
        Ptr<NetDevice> dev = mesh.devices.Get(i);
        Ptr<TrafficControlLayer> tc = mesh.nodes.Get(i)->GetObject<TrafficControlLayer>();
        if (tc)
        {
            Ptr<QueueDisc> root = tc->GetRootQueueDiscOnDevice(dev);
            if (root)
            {
                qdisc += CountDisc(root);
            }
        }
        Ptr<WifiNetDevice> wifi = DynamicCast<WifiNetDevice>(dev);
        if (wifi && wifi->GetMac())
        {
            Ptr<WifiMacQueue> q = wifi->GetMac()->GetTxopQueue(AC_BE);
            if (q)
            {
                mac += q->GetNPackets();
            }
        }
    }
    if (inQdiscs)
    {
        *inQdiscs = qdisc;
    }
    if (inRadios)
    {
        *inRadios = mac;
    }
    return qdisc + mac;
}

void
PrintHeldPackets(const MeshTopology& mesh)
{
    uint32_t qdisc = 0, mac = 0;
    uint32_t total = HeldPackets(mesh, &qdisc, &mac);
    cout << "  still held at end: " << qdisc << " in queue discs, " << mac << " in radio queues, "
         << total << " total\n";
}

namespace {

double g_drainStart = 0.0;
double g_drainTook = -1.0;

// Poll rather than wait a fixed time: the drain is a CONDITION (the queues are empty), not a
// number. RFC 2544 asks for a drain -- "wait for two seconds for any residual frames to be
// received" -- but two seconds is a figure for a switch whose buffer holds milliseconds, and a
// token bucket here holds seconds. So keep the requirement and drop the constant: stop as soon as
// nothing is held, and report how long it took, which is the settling time the Winter Simulation
// Conference 2003 checklist says must be stated.
void
DrainPoll(const MeshTopology& mesh, double pollSec, double capSec)
{
    double now = Simulator::Now().GetSeconds();
    if (HeldPackets(mesh, nullptr, nullptr) == 0)
    {
        g_drainTook = now - g_drainStart;
        Simulator::Stop(Seconds(0));
        return;
    }
    if (now - g_drainStart >= capSec)
    {
        Simulator::Stop(Seconds(0)); // cap reached with packets still held -> run is invalid
        return;
    }
    Simulator::Schedule(Seconds(pollSec), &DrainPoll, mesh, pollSec, capSec);
}

} // namespace

void
ScheduleDrain(const MeshTopology& mesh, double startSec, double pollSec, double capSec)
{
    g_drainStart = startSec;
    g_drainTook = -1.0;
    Simulator::Schedule(Seconds(startSec), &DrainPoll, mesh, pollSec, capSec);
}

bool
PrintDrain(const MeshTopology& mesh, double capSec)
{
    uint32_t held = HeldPackets(mesh, nullptr, nullptr);
    if (held == 0)
    {
        cout << "  drain: queues empty after " << g_drainTook << " s (condition, not a constant)\n";
        return true;
    }
    cout << "  drain: INVALID -- " << held << " packet(s) still held after the " << capSec
         << " s cap; every delay and loss figure in this run is truncated\n";
    return false;
}

void
PrintLossLedger()
{
    static const array<const char*, 4> macName = {"queue full (failed enqueue)",
                                                  "lifetime expired (500 ms MSDU)",
                                                  "retry limit reached",
                                                  "stale QoS packet"};
    cout << "\n=== where the medium lost frames (all nodes, all slices) ===\n";
    uint64_t total = 0;
    for (size_t i = 0; i < g_macDrops.size(); ++i)
    {
        cout << "  MAC " << macName[i] << ": " << g_macDrops[i] << "\n";
        total += g_macDrops[i];
    }
    cout << "  ARP cache drops (retries exhausted): " << g_arpDrops << "\n";
    cout << "  ARP pending-queue full (unresolvable next hop): " << g_arpQueueDrops << "\n";
    cout << "  IP drops (route/other): " << g_ipDrops << "\n";
    for (size_t i = 0; i < g_ipReason.size(); ++i)
    {
        if (g_ipReason[i])
        {
            cout << "    ip reason " << i << ": " << g_ipReason[i] << "\n";
        }
    }
    cout << "  MAC total: " << total << "\n";
}
