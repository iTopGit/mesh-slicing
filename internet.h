#ifndef INTERNET_H
#define INTERNET_H

#include "topology.h"

#include "ns3/ipv4-interface-container.h"

/**
 * Installs the IP stack (L3+L4: Ipv4L3Protocol, UdpL4Protocol, TcpL4Protocol) plus
 * OLSR (L3 multi-hop routing) on every node in `mesh.nodes`, and assigns an address
 * to every device in `mesh.devices`. Returns the interface container so applications
 * can look up a node's address by index (same order as mesh.nodes/mesh.devices).
 */
ns3::Ipv4InterfaceContainer InstallInternetStack(const MeshTopology& mesh);

#endif // INTERNET_H
