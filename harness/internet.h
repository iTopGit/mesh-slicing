#ifndef INTERNET_H
#define INTERNET_H

#include "topology.h"

#include "../nsga/routing.h" // nsga::Route (GA next-hop table)

#include "ns3/ipv4-interface-container.h"
#include "ns3/queue-disc-container.h"

#include <string>
#include <vector>

/**
 * Installs the IP stack (L3+L4: Ipv4L3Protocol, UdpL4Protocol, TcpL4Protocol) on every
 * node in `mesh.nodes`, assigns an address to every device, and returns the interface
 * container (index order matches mesh.nodes/mesh.devices). `useOlsr` picks the routing
 * method — the two are never mixed, one per comparison run:
 *   false (default) : pure Ipv4StaticRouting. Connected subnet routes are auto-added, so
 *                     single-hop neighbours deliver directly; injected /32 host routes
 *                     (GA / Seng) win multi-hop by longest prefix. No OLSR chatter to tax
 *                     the warm-up measurement.
 *   true            : OLSR proactive routing — the standalone-protocol comparison baseline.
 *
 * If `sliceQdiscs` is non-null the per-slice queue disc goes on every radio and the
 * container is filled with one root disc per device. It has to happen here rather than in
 * the caller: the discs must be installed between the stack install and the address
 * assignment (see InstallSliceQdiscs), and both of those live inside this function.
 *
 * Otherwise `plainQdisc` goes on every radio, at its own default depth. Default is
 * `ns3::FqCoDelQueueDisc` because that is what Linux ships, so the no-slicing arm needs no
 * argument for its queue: it is the one a real deployment would already have. That also makes it
 * the harder baseline to beat, which is the comparison a reviewer will ask for -- fair queueing
 * plus CoDel already separates flows and holds delay down without any slicing. `ns3::FifoQueueDisc`
 * is the optional no-queue-management arm, useful only to show how far a dumb buffer falls behind.
 *
 * Warm-up no longer cares which one is installed: it resets the sender between links, so the
 * measured table came out identical on both (median 82.7244 Mb/s, 40/40 links, 0 patched).
 */
ns3::Ipv4InterfaceContainer InstallInternetStack(const MeshTopology& mesh,
                                                 bool useOlsr = false,
                                                 ns3::QueueDiscContainer* sliceQdiscs = nullptr,
                                                 const std::string& plainQdisc = "ns3::FqCoDelQueueDisc");

/**
 * Install the GA's next-hop table as static host routes on the mesh nodes. Each entry
 * becomes `Ipv4StaticRouting::AddHostRouteTo(dest, nextHop)` on node `r.node`; these
 * out-rank OLSR (see InstallInternetStack). Call AFTER the routes are computed.
 */
void InjectStaticRoutes(const MeshTopology& mesh,
                        const ns3::Ipv4InterfaceContainer& interfaces,
                        const std::vector<nsga::Route>& routes);

/**
 * "Perfect ARP": give every node a PERMANENT cache entry for every other node, so no lookup ever
 * misses and no ARP frame is ever sent. Part of warm-up -- the mesh is brought to a known state
 * before anything is measured, the same principle as resetting the sender between link probes.
 *
 * The name is ns-3's own, and MarkPermanent() was added for it (bug 2145). There is no bulk
 * helper, so the loop is written out (issue 311 is an open request for one).
 *
 * Two references make it practice rather than preference. RFC 2544 4.1.1 asks for learning frames
 * ahead of a trial so the measurement is of forwarding. And Perrone, Yuan & Nicol (Winter
 * Simulation Conference 2003) name this defect exactly: ARP holds only ONE packet per unresolved
 * address and drops the rest, they measured it at under 6% of delivery in a stationary network,
 * and they note that few papers even state whether their stack includes ARP. Here it was 1109
 * packets of 9996 -- worse, because 24 flows start at once -- and one flow delivered NOTHING for
 * the whole run, its entry having gone DEAD for a DeadTimeout (100 s) longer than the traffic.
 */
void PerfectArp(const MeshTopology& mesh, const ns3::Ipv4InterfaceContainer& interfaces);

#endif // INTERNET_H
