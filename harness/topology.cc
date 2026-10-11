#include "topology.h"

#include "ns3/double.h"
#include "ns3/mobility-helper.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/yans-wifi-helper.h"

#include <sstream>

using namespace ns3;
using namespace std;

MeshTopology
BuildGridAdhocAxTopology(uint32_t gridWidth,
                         uint32_t gridHeight,
                         double spacing,
                         uint32_t chWidth,
                         const string& pcapPrefix)
{
    NodeContainer nodes;
    nodes.Create(gridWidth * gridHeight);

    // Static grid, row by row from (0,0). Positions set signal strength.
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue(0.0),
                                   "MinY", DoubleValue(0.0),
                                   "DeltaX", DoubleValue(spacing),
                                   "DeltaY", DoubleValue(spacing),
                                   "GridWidth", UintegerValue(gridWidth),
                                   "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    // One shared channel, LogDistance propagation so rate/capacity degrades with
    // distance (PL(d) dB = 46.68 + 30*log10(d), default exponent/ref-loss).
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    ostringstream ch;
    ch << "{0, " << chWidth << ", BAND_5GHZ, 0}";
    phy.Set("ChannelSettings", StringValue(ch.str()));

    // 20 dBm, not ns-3's 16.02 default: it is the highest permitted in Europe and the value
    // "5G Network Slicing for Wi-Fi Networks" fixes for every slice. It also reproduces the
    // ~80-100 Mb/s per link that the Seng mesh paper reports at this 50 m spacing, where the
    // ns-3 default lands at roughly half that.
    phy.Set("TxPowerStart", DoubleValue(20.0));
    phy.Set("TxPowerEnd", DoubleValue(20.0));

    // Defer only to nodes we share an edge with. At 20 dBm an axis neighbour (50 m) arrives at
    // -77.65 dBm and a diagonal (70.7 m) at -82.15, so the threshold goes midway, -79.9; the
    // attribute is 6 dB lower because ns-3 scales it by 10*log10(width/20MHz) for an 80 MHz
    // channel (yans-wifi-channel.cc). Left at the -101 default a node hears ~189 m across a
    // 200 m grid, so the whole field takes turns and the medium carries one link's worth.
    phy.Set("RxSensitivity", DoubleValue(-86.0));

    // 802.11ax, ad-hoc mode (no AP, no association). IdealWifiManager picks the MCS
    // from known SNR — no adaptive-rate settling to wait out at warm-up.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    if (!pcapPrefix.empty())
    {
        phy.EnablePcap(pcapPrefix, devices);
    }

    return {nodes, devices};
}
