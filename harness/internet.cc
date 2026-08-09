#include "internet.h"

#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/olsr-helper.h"

using namespace ns3;

Ipv4InterfaceContainer
InstallInternetStack(const MeshTopology& mesh)
{
    // L3+L4 on every node, one flat subnet across the whole grid. The ax ad-hoc
    // radios have no L2 routing, so OLSR does the multi-hop at L3. NSGA-II routes
    // are pushed on top as static routes (Ipv4StaticRouting); OLSR is the default/
    // baseline router for anything not overridden.
    InternetStackHelper internet;
    OlsrHelper olsr;
    internet.SetRoutingHelper(olsr);
    internet.Install(mesh.nodes);

    // One /16 covers all gridWidth^2 nodes with room to spare.
    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.0.0");
    return address.Assign(mesh.devices);
}
