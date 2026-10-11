#include "internet.h"

#include "traffic-control.h" // InstallSliceQdiscs (must run before the addresses are assigned)

#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/arp-cache.h"
#include "ns3/arp-l3-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4.h"
#include "ns3/olsr-helper.h"
#include "ns3/string.h"
#include "ns3/traffic-control-helper.h"

using namespace ns3;
using namespace std;

Ipv4InterfaceContainer
InstallInternetStack(const MeshTopology& mesh,
                     bool useOlsr,
                     QueueDiscContainer* sliceQdiscs,
                     const string& plainQdisc)
{
    // The ax ad-hoc radios have no L2 routing, so L3 provides the multi-hop paths. Two
    // routing methods, never mixed (one per comparison run): OLSR (protocol baseline) OR
    // pure static (GA / Seng inject host routes; native + warm-up run on this, no chatter).
    InternetStackHelper internet;
    if (useOlsr)
    {
        OlsrHelper olsr;
        internet.SetRoutingHelper(olsr);
    }
    else
    {
        Ipv4StaticRoutingHelper staticRh;
        internet.SetRoutingHelper(staticRh);
    }
    internet.Install(mesh.nodes);

    // Before Assign, never after: Assign installs the default queue disc into an empty slot,
    // and TrafficControlLayer wires each disc's SendCallback once, during initialization.
    if (sliceQdiscs)
    {
        *sliceQdiscs = InstallSliceQdiscs(mesh.devices);
    }
    else
    {
        // Named explicitly rather than left to Assign's default, which is FqCoDel and which
        // nobody chose. FqCoDel also masked half of the warm-up measurement bug: it flow-queues,
        // so a new link got its own lane instead of queueing behind the previous link's backlog.
        TrafficControlHelper tch;
        tch.SetRootQueueDisc(plainQdisc);
        tch.Install(mesh.devices);
    }

    // One /16 covers all gridWidth^2 nodes with room to spare.
    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.0.0");
    return address.Assign(mesh.devices);
}

void
InjectStaticRoutes(const MeshTopology& mesh,
                   const Ipv4InterfaceContainer& interfaces,
                   const vector<nsga::Route>& routes)
{
    Ipv4StaticRoutingHelper srh;
    for (const nsga::Route& r : routes)
    {
        Ptr<Ipv4StaticRouting> sr = srh.GetStaticRouting(mesh.nodes.Get(r.node)->GetObject<Ipv4>());
        // Interface 1 = the wifi device (0 is loopback), same as the topology assign order.
        sr->AddHostRouteTo(interfaces.GetAddress(r.dest), interfaces.GetAddress(r.nextHop), 1);
    }
}

void
PerfectArp(const MeshTopology& mesh, const Ipv4InterfaceContainer& interfaces)
{
    // "Perfect ARP" is ns-3's own name for this: every lookup hits the cache, so no ARP frame is
    // ever generated and address resolution leaves the measurement. MarkPermanent() exists for
    // exactly this (ns-3 bug 2145); there is no helper to do it in bulk, which is why this loop
    // is written out (ns-3 issue 311 is an open request for one).
    //
    // Two citations make it practice rather than preference. RFC 2544 4.1.1 asks for learning
    // frames before a trial so the result is about forwarding. And Perrone, Yuan & Nicol, Winter
    // Simulation Conference 2003, 4, describe this precise defect: "ARP buffers only one
    // outstanding packet for each IP address that is waiting for a query resolution. If more
    // packets are presented for this same address, they are dropped by ARP." They measured its
    // cost on delivery at under 6% in a stationary network; here it was 1109 packets of 9996,
    // worse because 24 flows start at once, and one flow lost 100% of its traffic for the whole
    // run because its entry went DEAD and DeadTimeout (100 s) outlasts the traffic window.
    //
    // A real mesh does not pay this either: OLSR and HWMP keep neighbour tables up continuously,
    // so an established mesh forwards without a resolution round per flow.
    for (uint32_t i = 0; i < mesh.nodes.GetN(); ++i)
    {
        Ptr<Ipv4> ipv4 = mesh.nodes.Get(i)->GetObject<Ipv4>();
        int32_t ifIndex = ipv4 ? ipv4->GetInterfaceForDevice(mesh.devices.Get(i)) : -1;
        if (ifIndex < 0)
        {
            continue;
        }
        Ptr<Ipv4Interface> iface =
            DynamicCast<Ipv4Interface>(ipv4->GetObject<Ipv4L3Protocol>()->GetInterface(ifIndex));
        Ptr<ArpCache> cache = iface ? iface->GetArpCache() : nullptr;
        if (!cache)
        {
            continue;
        }
        for (uint32_t j = 0; j < mesh.nodes.GetN(); ++j)
        {
            if (j == i)
            {
                continue;
            }
            ArpCache::Entry* e = cache->Add(interfaces.GetAddress(j));
            e->SetMacAddress(Mac48Address::ConvertFrom(mesh.devices.Get(j)->GetAddress()));
            e->MarkPermanent();
        }
    }
}

