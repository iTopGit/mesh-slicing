#include "applications.h"

#include "ns3/core-module.h"
#include "ns3/inet-socket-address.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/udp-echo-helper.h"

using namespace ns3;

void
InstallEchoApps(const MeshTopology& mesh,
                 const Ipv4InterfaceContainer& interfaces,
                 uint32_t src,
                 uint32_t dst)
{
    uint16_t port = 9;

    UdpEchoServerHelper server(port);
    ApplicationContainer serverApp = server.Install(mesh.nodes.Get(dst));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(5.0));

    UdpEchoClientHelper client(interfaces.GetAddress(dst), port);
    client.SetAttribute("MaxPackets", UintegerValue(3));
    client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    client.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApp = client.Install(mesh.nodes.Get(src));
    // Starts at 1s to give OLSR a moment to converge on the multi-hop route.
    clientApp.Start(Seconds(1.0));
    clientApp.Stop(Seconds(5.0));
}

void
InstallSliceTraffic(const MeshTopology& mesh,
                    const Ipv4InterfaceContainer& interfaces,
                    const Slice& slice,
                    double startSec,
                    double stopSec)
{
    const uint32_t pktSize = 700; // proposal slice payload

    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(interfaces.GetAddress(slice.dst), slice.port));
    onoff.SetConstantRate(DataRate(slice.dataRate), pktSize);
    // Tos -> socket IpTos -> packet priority, marking the slice for per-flow reporting
    // / classification. (SourceApplication "Tos" attribute; no ToS on the address.)
    onoff.SetAttribute("Tos", UintegerValue(slice.tos));
    ApplicationContainer src = onoff.Install(mesh.nodes.Get(slice.src));
    src.Start(Seconds(startSec));
    src.Stop(Seconds(stopSec));

    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), slice.port));
    ApplicationContainer rcv = sink.Install(mesh.nodes.Get(slice.dst));
    rcv.Start(Seconds(startSec));
    rcv.Stop(Seconds(stopSec + 1.0));
}
