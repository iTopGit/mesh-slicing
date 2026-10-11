// Gate: does injecting NSGA-II-style next-hop routes actually FORCE traffic along
// the chosen path, and does the *GA's own* output route real traffic end-to-end?
// A 4-neighbour grid (adjacency forced by propagation range), no OLSR, so injected
// routes are the ONLY routes.
//   default    : install a deliberately long HARDCODED path, trace forwarding, and
//                confirm packets took exactly the injected relays (step 1).
//   --fromGa   : run the GA, BuildRoutes on the knee pick, inject via the SAME
//                InjectRoutes, then send per-slice traffic and confirm every slice
//                delivers (step 2 — the nsga -> harness wire, end to end).

#include "../nsga/nsga2.h"
#include "../nsga/routing.h" // Route struct + BuildRoutes (also pulls graph/fitness/individual)

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <numeric>
#include <random>
#include <set>

using namespace ns3;
using namespace std;
using namespace nsga; // Route, Graph, Individual, Slice, Run, KneePoint, BuildRoutes

static set<uint32_t> g_forwarded; // node ids that relayed (IP-forwarded) a packet

static void
OnForward(uint32_t nodeId, const Ipv4Header&, Ptr<const Packet>, uint32_t)
{
    g_forwarded.insert(nodeId);
}

// The reusable inject step: turn a next-hop table (node/dest/nextHop as node indices)
// into ns-3 static routes. Both modes call this; step 2 hands it BuildRoutes' output.
static void
InjectRoutes(const NodeContainer& nodes, const Ipv4InterfaceContainer& iface,
             const vector<Route>& routes)
{
    Ipv4StaticRoutingHelper srh;
    for (const Route& r : routes)
    {
        Ptr<Ipv4StaticRouting> sr = srh.GetStaticRouting(nodes.Get(r.node)->GetObject<Ipv4>());
        sr->AddHostRouteTo(iface.GetAddress(r.dest), iface.GetAddress(r.nextHop), 1);
    }
}

// One short UDP flow src->dst on its own port; returns the sink to read delivery.
static Ptr<PacketSink>
AddFlow(const NodeContainer& nodes, const Ipv4InterfaceContainer& iface, uint32_t src, uint32_t dst,
        uint16_t port, const string& rate)
{
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(iface.GetAddress(dst), port));
    onoff.SetConstantRate(DataRate(rate), 700);
    ApplicationContainer a = onoff.Install(nodes.Get(src));
    a.Start(Seconds(1.0));
    a.Stop(Seconds(4.0));
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer r = sink.Install(nodes.Get(dst));
    r.Start(Seconds(0.0));
    r.Stop(Seconds(5.0));
    return DynamicCast<PacketSink>(r.Get(0));
}

int
main(int argc, char* argv[])
{
    uint32_t gridWidth = 4;
    double spacing = 50.0;
    double maxRange = 60.0; // 50 m (orthogonal) in range, 70.7 m (diagonal) blocked
    uint32_t chWidth = 80;
    bool fromGa = false;
    uint32_t Me = 3, Mu = 3, Mc = 3;
    double embb = 2.0, urllc = 2.0, mmtc = 2.0;
    uint32_t pop = 40, gens = 20, seed = 2;
    CommandLine cmd;
    cmd.AddValue("gridWidth", "Grid width (square grid)", gridWidth);
    cmd.AddValue("spacing", "Distance between adjacent nodes (m)", spacing);
    cmd.AddValue("chWidth", "Channel width (MHz)", chWidth);
    cmd.AddValue("fromGa", "Feed the GA's routes instead of a hardcoded path", fromGa);
    cmd.AddValue("Me", "eMBB member count", Me);
    cmd.AddValue("Mu", "URLLC member count", Mu);
    cmd.AddValue("Mc", "mMTC member count", Mc);
    cmd.AddValue("pop", "GA population", pop);
    cmd.AddValue("gens", "GA generations", gens);
    cmd.AddValue("seed", "RNG seed", seed);
    cmd.Parse(argc, argv);

    uint32_t n = gridWidth * gridWidth;
    NodeContainer nodes;
    nodes.Create(n);

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
    mobility.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY",
                                  DoubleValue(0.0), "DeltaX", DoubleValue(spacing), "DeltaY",
                                  DoubleValue(spacing), "GridWidth", UintegerValue(gridWidth),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    // Default stack: static routing available, NO OLSR -> injected routes are the only routes.
    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.0.0");
    Ipv4InterfaceContainer iface = addr.Assign(dev);

    for (uint32_t i = 0; i < n; ++i)
    {
        Ptr<Ipv4L3Protocol> l3 = nodes.Get(i)->GetObject<Ipv4L3Protocol>();
        l3->TraceConnectWithoutContext("UnicastForward", MakeBoundCallback(&OnForward, i));
    }

    // ---- step 2: feed the GA's routes ----
    if (fromGa)
    {
        if (Me + Mu + Mc > n)
        {
            cout << "error: Me+Mu+Mc (" << (Me + Mu + Mc) << ") exceeds nodes (" << n << ")\n";
            return 1;
        }
        // Graph adjacency (orthogonal grid) MATCHES the range-forced mesh, so every
        // route the GA emits is a valid in-range hop on the real radios.
        Graph gr = Graph::FromGrid(gridWidth, gridWidth, 0.245, 51.0);
        mt19937 rng(seed);
        vector<uint32_t> pool(n);
        iota(pool.begin(), pool.end(), 0);
        shuffle(pool.begin(), pool.end(), rng);
        array<Slice, NUM_SLICES> slices;
        slices[0] = {.name = "eMBB", .members = {pool.begin(), pool.begin() + Me}, .demandMbps = embb};
        slices[1] = {.name = "URLLC", .members = {pool.begin() + Me, pool.begin() + Me + Mu}, .demandMbps = urllc};
        slices[2] = {.name = "mMTC", .members = {pool.begin() + Me + Mu, pool.begin() + Me + Mu + Mc}, .demandMbps = mmtc};

        Params params{pop, gens, 0.1, 4.53};
        vector<Individual> front = Run(gr, slices, params, rng);
        if (front.empty())
        {
            cout << "error: GA returned an empty Pareto front\n";
            return 1;
        }
        size_t knee = KneePoint(front);
        vector<Route> routes = BuildRoutes(gr, front[knee], slices);
        InjectRoutes(nodes, iface, routes);

        cout << "\n=== route-inject gate (FROM GA, " << gridWidth << "x" << gridWidth << " grid) ===\n";
        cout << "knee solution: eMBB_tp=" << front[knee].obj[0] << " URLLC_delay=" << front[knee].obj[1]
             << " mMTC_links=" << (int)front[knee].obj[2] << "\n";
        cout << "installed " << routes.size() << " routes across 3 slices\n\n";

        // Per slice, send member[0] -> member[last] over its own routes.
        const char* sn[NUM_SLICES] = {"eMBB ", "URLLC", "mMTC "};
        struct Probe { int slice; uint32_t src, dst; Ptr<PacketSink> sink; };
        vector<Probe> probes;
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            if (slices[s].members.size() < 2)
            {
                continue;
            }
            uint32_t src = slices[s].members.front();
            uint32_t dst = slices[s].members.back();
            probes.push_back({s, src, dst, AddFlow(nodes, iface, src, dst, 5001 + s, "2Mb/s")});
        }

        Simulator::Stop(Seconds(5.0));
        Simulator::Run();

        bool allPass = true;
        cout << "slice   src -> dst   delivered\n";
        for (const Probe& p : probes)
        {
            double mb = p.sink->GetTotalRx() * 8.0 / 3.0 / 1e6;
            bool ok = p.sink->GetTotalRx() > 0;
            allPass = allPass && ok;
            cout << sn[p.slice] << "   " << p.src << " -> " << p.dst << "     " << mb << " Mb/s  "
                 << (ok ? "OK" : "FAIL") << "\n";
        }
        cout << (allPass ? "PASS" : "FAIL") << ": the GA's routes carry every slice's traffic\n";
        Simulator::Destroy();
        return allPass ? 0 : 1;
    }

    // ---- step 1: hardcoded forced path ----
    // Deliberately LONG path to dest 10 (the 4-hop shortest is avoided): if traffic
    // takes this route, the injected next hops — not geography — decided the path.
    uint32_t dst = 10;
    vector<uint32_t> path = {0, 1, 2, 3, 7, 11, 10};
    vector<Route> routes;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        routes.push_back({path[i], dst, path[i + 1], 0});
    }
    InjectRoutes(nodes, iface, routes);

    Ptr<PacketSink> sk = AddFlow(nodes, iface, 0, dst, 5000, "5Mb/s");

    Simulator::Stop(Seconds(5.0));
    Simulator::Run();

    set<uint32_t> expected(path.begin() + 1, path.end() - 1); // relays only (drop src, dst)
    double rxMb = sk->GetTotalRx() * 8.0 / 3.0 / 1e6;
    cout << "\n=== route-inject gate (" << gridWidth << "x" << gridWidth << " grid) ===\n";
    cout << "injected path: ";
    for (uint32_t x : path)
    {
        cout << x << (x == dst ? "" : " -> ");
    }
    cout << "   (dest " << dst << ")\n";
    cout << "delivered:   " << rxMb << " Mb/s  (rx bytes " << sk->GetTotalRx() << ")\n";
    cout << "forwarded by:";
    for (uint32_t x : g_forwarded)
    {
        cout << " " << x;
    }
    cout << "\nexpected relays:";
    for (uint32_t x : expected)
    {
        cout << " " << x;
    }
    cout << "\n";
    bool pass = sk->GetTotalRx() > 0 && g_forwarded == expected;
    cout << (pass ? "PASS" : "FAIL") << ": injected routes "
         << (pass ? "forced the exact path" : "did NOT force the path") << "\n";

    Simulator::Destroy();
    return pass ? 0 : 1;
}
