// PLAN.md REQUIRED GATE: does the contention formula match ns-3 on one shared link?
//
// Model (PLAN.md §3/§4, PopJoi.md), corrected 2026-08-06: per slice on a link
// a = D/R (airtime fraction), U = Sum a over the slices sharing it; predicted
// throughput = max-min fair share of R across the offered demands (measured
// equal-split at U=3, not the old proportional D/U guess); predicted delay
// L_eff = C / (1 - U), C = calibrated loaded-link constant (not L_base — see
// PLAN.md §10 "Formula fix").
//
// One link = 2 ax ad-hoc nodes. In one run: (A) light flow -> L_base, (B) saturating
// flow -> R, (C) 3 slice flows sharing the link -> measured per-slice tx/delay. Then
// print measured vs formula so the two can be compared across a sweep of offered loads.

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <iomanip>
#include <map>
#include <vector>

using namespace ns3;
using namespace std;

// Max-min fair share of capacity R across demand[]: a slice under the equal
// split gets its full ask; leftover capacity re-splits evenly among the rest.
static vector<double>
FairShare(double R, const vector<double>& demand)
{
    vector<double> alloc(demand.size(), 0.0);
    vector<bool> settled(demand.size(), false);
    double remaining = R;
    size_t nActive = demand.size();
    bool changed = true;
    while (changed && nActive > 0)
    {
        changed = false;
        double share = remaining / nActive;
        for (size_t i = 0; i < demand.size(); ++i)
        {
            if (!settled[i] && demand[i] <= share)
            {
                alloc[i] = demand[i];
                remaining -= demand[i];
                settled[i] = true;
                nActive--;
                changed = true;
            }
        }
    }
    if (nActive > 0)
    {
        double share = remaining / nActive;
        for (size_t i = 0; i < demand.size(); ++i)
        {
            if (!settled[i])
            {
                alloc[i] = share;
            }
        }
    }
    return alloc;
}

// One UDP flow src->dst at a constant rate on its own port, over a time window.
static void
AddFlow(Ptr<Node> src, Ptr<Node> dst, Ipv4Address dstAddr, uint16_t port,
        const string& rate, uint32_t pktSize, double start, double stop)
{
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    onoff.SetConstantRate(DataRate(rate), pktSize);
    ApplicationContainer s = onoff.Install(src);
    s.Start(Seconds(start));
    s.Stop(Seconds(stop));

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer r = sink.Install(dst);
    r.Start(Seconds(start - 0.5));
    r.Stop(Seconds(stop + 0.5));
}

int
main(int argc, char* argv[])
{
    uint32_t chWidth = 80;
    double spacing = 50.0;    // match the mesh-slicing grid so R is comparable
    uint32_t pktSize = 700;   // proposal slice payload
    // Three slice offered rates (Mb/s). Sweep these from a driver to move U across 1.
    double d1 = 20.0, d2 = 15.0, d3 = 10.0;
    double C = 4.53; // calibrated loaded-link delay constant (ms), avg of 5 pts U=0.27-0.79; PLAN.md §10
    CommandLine cmd;
    cmd.AddValue("chWidth", "Channel width (MHz)", chWidth);
    cmd.AddValue("spacing", "Link length (m)", spacing);
    cmd.AddValue("d1", "eMBB offered Mb/s", d1);
    cmd.AddValue("d2", "URLLC offered Mb/s", d2);
    cmd.AddValue("d3", "mMTC offered Mb/s", d3);
    cmd.AddValue("C", "Calibrated loaded-link delay constant (ms)", C);
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(2);

    MobilityHelper mob;
    mob.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0),
                             "MinY", DoubleValue(0.0), "DeltaX", DoubleValue(spacing),
                             "DeltaY", DoubleValue(0.0), "GridWidth", UintegerValue(2),
                             "LayoutType", StringValue("RowFirst"));
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
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

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer iface = addr.Assign(dev);
    Ipv4Address dst = iface.GetAddress(1);

    // Ports: 4000 = L_base probe, 4001 = R probe, 5001/2/3 = the 3 slices.
    AddFlow(nodes.Get(0), nodes.Get(1), dst, 4000, "1Mb/s", pktSize, 1.0, 2.0);   // L_base
    AddFlow(nodes.Get(0), nodes.Get(1), dst, 4001, "300Mb/s", pktSize, 3.0, 5.0); // R (saturate)
    auto mbps = [](double m) { return to_string(m) + "Mb/s"; };
    AddFlow(nodes.Get(0), nodes.Get(1), dst, 5001, mbps(d1), pktSize, 6.0, 9.0);
    AddFlow(nodes.Get(0), nodes.Get(1), dst, 5002, mbps(d2), pktSize, 6.0, 9.0);
    AddFlow(nodes.Get(0), nodes.Get(1), dst, 5003, mbps(d3), pktSize, 6.0, 9.0);

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();

    Simulator::Stop(Seconds(10.0));
    Simulator::Run();

    // Collect per-port throughput (Mb/s over the flow's active time) and mean delay (ms).
    map<uint16_t, double> tp, delay;
    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    for (const auto& [id, st] : fm->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple ft = cl->FindFlow(id);
        if (st.rxPackets == 0)
        {
            continue;
        }
        double dur = (st.timeLastRxPacket - st.timeFirstRxPacket).GetSeconds();
        tp[ft.destinationPort] = (dur > 0) ? st.rxBytes * 8.0 / dur / 1e6 : 0.0;
        delay[ft.destinationPort] = st.delaySum.GetSeconds() / st.rxPackets * 1000.0;
    }

    double Lbase = delay[4000];       // ms
    double R = tp[4001];              // Mb/s, saturating single-flow capacity
    double U = (d1 + d2 + d3) / R;    // total airtime demand fraction

    cout << fixed << setprecision(3);
    cout << "\n=== formula gate (one ax " << chWidth << "MHz link, " << spacing << "m) ===\n";
    cout << "calibrated:  R = " << R << " Mb/s   L_base = " << Lbase << " ms\n";
    cout << "offered:     d1=" << d1 << "  d2=" << d2 << "  d3=" << d3
              << " Mb/s   ->  U = (d1+d2+d3)/R = " << U << "\n\n";

    double predDelay = (U < 1.0) ? C / (1.0 - U) : -1.0; // -1 => saturated
    vector<double> predTpVec = FairShare(R, {d1, d2, d3});
    cout << "slice   offered   pred_tp   meas_tp   pred_delay   meas_delay\n";
    struct S { const char* name; double d; uint16_t port; };
    S slices[] = {{"eMBB ", d1, 5001}, {"URLLC", d2, 5002}, {"mMTC ", d3, 5003}};
    for (size_t i = 0; i < 3; ++i)
    {
        const S& s = slices[i];
        cout << s.name << "   " << setw(6) << s.d << "    " << setw(6) << predTpVec[i]
                  << "    " << setw(6) << tp[s.port] << "    ";
        if (predDelay < 0)
        {
            cout << setw(9) << "SAT" << "    ";
        }
        else
        {
            cout << setw(9) << predDelay << "    ";
        }
        cout << setw(9) << delay[s.port] << "\n";
    }
    double measTot = tp[5001] + tp[5002] + tp[5003];
    double predTot = predTpVec[0] + predTpVec[1] + predTpVec[2];
    cout << "\ntotal tp: predicted " << predTot << "  measured " << measTot << " Mb/s\n";

    // Back-solve this run's own calibration point: C_calib = avg(meas_delay) * (1-U).
    // Collect several runs across U in ~0.3-0.85, average C_calib -> plug into --C above.
    if (U < 1.0)
    {
        double avgMeasDelay = (delay[5001] + delay[5002] + delay[5003]) / 3.0;
        double cCalib = avgMeasDelay * (1.0 - U);
        cout << "calib: avg_meas_delay=" << avgMeasDelay << " ms   U=" << U
                  << "   C_calib = avg_meas_delay*(1-U) = " << cCalib << " ms\n";
    }

    Simulator::Destroy();
    return 0;
}
