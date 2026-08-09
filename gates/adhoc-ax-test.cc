// Gate test (PLAN.md "Alternative architecture"): does 802.11ax ad-hoc do
// MULTI-HOP in ns-3, with routes installed as L3 static routes (the exact
// mechanism NSGA-II would use)? A line of N nodes is forced by propagation
// range (adjacent in range, non-adjacent blocked); traffic goes end-to-end
// through the relays. High end-to-end throughput = the ax-adhoc + L3-routing
// architecture works.

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

using namespace ns3;
using namespace std;

int
main(int argc, char* argv[])
{
    uint32_t nNodes = 3;
    uint32_t chWidth = 80;  // MHz
    double spacing = 40.0;  // m between adjacent nodes
    double maxRange = 50.0; // m: adjacent (40) in range, 2-hop (80) blocked -> forced line
    CommandLine cmd;
    cmd.AddValue("nNodes", "Nodes in the line", nNodes);
    cmd.AddValue("chWidth", "Channel width (MHz)", chWidth);
    cmd.AddValue("spacing", "Distance between adjacent nodes (m)", spacing);
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(nNodes);

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(maxRange));
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    ostringstream ch;
    ch << "{0, " << chWidth << ", BAND_5GHZ, 0}";
    phy.Set("ChannelSettings", StringValue(ch.str()));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer dev = wifi.Install(phy, mac, nodes);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(spacing),
                                  "DeltaY", DoubleValue(0.0),
                                  "GridWidth", UintegerValue(nNodes),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer iface = addr.Assign(dev);

    // NSGA-II-style route injection: install the line path as L3 static routes.
    // Each node reaches a non-adjacent destination via the neighbor toward it.
    Ipv4StaticRoutingHelper srh;
    for (uint32_t i = 0; i < nNodes; ++i)
    {
        Ptr<Ipv4StaticRouting> r = srh.GetStaticRouting(nodes.Get(i)->GetObject<Ipv4>());
        for (uint32_t j = 0; j < nNodes; ++j)
        {
            if (i == j)
            {
                continue;
            }
            uint32_t next = (j > i) ? i + 1 : i - 1;
            if (next != j) // non-adjacent -> route via the neighbor; adjacent handled by subnet
            {
                r->AddHostRouteTo(iface.GetAddress(j), iface.GetAddress(next), 1);
            }
        }
    }

    // 3 prioritized flows 0 -> last, each congesting, different IP ToS -> Wi-Fi AC.
    // Static routes (no OLSR), so the ONLY thing that could differentiate them is
    // the MAC's EDCA. If VO delivers more than BK, EDCA works on ax ad-hoc.
    struct Flow { const char* name; uint8_t tos; uint16_t port; };
    Flow flows[] = {{"URLLC(VO)", 0x10, 5001}, {"eMBB(VI)", 0x18, 5002}, {"mMTC(BK)", 0x08, 5003}};
    vector<Ptr<PacketSink>> sinks;
    for (const Flow& f : flows)
    {
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(iface.GetAddress(nNodes - 1), f.port));
        onoff.SetConstantRate(DataRate("100Mb/s"), 1400);
        onoff.SetAttribute("Tos", UintegerValue(f.tos));
        ApplicationContainer src = onoff.Install(nodes.Get(0));
        src.Start(Seconds(1.0));
        src.Stop(Seconds(5.0));

        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), f.port));
        ApplicationContainer rcv = sink.Install(nodes.Get(nNodes - 1));
        rcv.Start(Seconds(0.0));
        rcv.Stop(Seconds(6.0));
        sinks.push_back(DynamicCast<PacketSink>(rcv.Get(0)));
    }

    Simulator::Stop(Seconds(6.0));
    Simulator::Run();

    cout << "ax ad-hoc " << chWidth << "MHz, " << (nNodes - 1)
              << " hops, 3x100Mb/s prioritized flows:" << endl;
    for (size_t i = 0; i < 3; ++i)
    {
        double mb = sinks[i]->GetTotalRx() * 8.0 / 4.0 / 1e6;
        cout << "  " << flows[i].name << ": " << mb << " Mb/s delivered" << endl;
    }

    Simulator::Destroy();
    return 0;
}
