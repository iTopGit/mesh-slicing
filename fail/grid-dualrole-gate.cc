// Gate T3b (DualChannel.md): the full harness geometry -- 5x5 grid, 50 m spacing -- built entirely
// out of dual-role nodes (1 AP radio + 1 STA radio each, 2 channels), carrying random traffic.
// The point is a stress/soak test: every node forwards, many BSSs share two channels, and nothing
// may crash. Rings (T3) proved the structure on cycles; this proves it at harness scale.
//
// Roles (--roles), the trade this gate exists to measure:
//   tree    : 1 AP + 1 STA per node. Each node runs its own BSS and joins ONE parent BSS (the grid
//             neighbour a step toward the centre). 2 radios, 2 channels. Only parent-child grid
//             links survive -- 24 of 40 on a 5x5, and that is the MAXIMUM for one STA radio, since
//             siblings of an AP are its neighbours and those are pairwise diagonal, never adjacent.
//   checker : black (x+y even) = AP, white = STA of EVERY adjacent black. All 40 grid links live,
//             but a white node needs one radio per neighbour -- up to 4 radios and 4 channels.
//
// Channels. tree: chan = (x+y) % 2; a parent is always a grid neighbour so it has opposite parity,
// which keeps a node's AP channel off its STA channel with 2 colours. checker: two black APs clash
// when they share a white neighbour, i.e. when they differ by (+-1,0), (0,+-1) or (+-1,+-1) in the
// rotated black lattice u=(x+y)/2, v=(x-y)/2 -- a king graph, so 4 colours: (u%2)*2 + (v%2).
//
// Routing: no GA. Members of one BSS share a subnet and all reach each other (the AP relays
// STA->STA), so the IP graph is a clique per BSS. Edge cost is RADIO TRANSMISSIONS, not IP hops:
// AP<->STA costs 1, STA<->STA costs 2 because the AP repeats the frame. Shortest paths come from
// Dijkstra on that cost and are installed as host routes. Resource allocation (who is AP, on which
// channel) is decided before and independently of this -- the separation the thesis is built on.
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/wifi-module.h"

#include <csignal>
#include <cmath>
#include <cstdlib>
#include <execinfo.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <queue>
#include <set>
#include <unistd.h>
#include <vector>

using namespace ns3;
using namespace std;

namespace {

// Non-overlapping 80 MHz centre channel numbers (36 is not a valid 80 MHz centre). tree uses the
// first two, checker all four.
const vector<uint32_t> CH80 = {42, 155, 58, 106};
const uint16_t PORT_BASE = 9000;

// No gdb in this environment: dump a native backtrace and the sim clock on SIGSEGV.
void
SegvHandler(int)
{
    void* bt[64];
    int n = backtrace(bt, 64);
    fprintf(stderr, "\n*** SIGSEGV at sim t = %.6f s ***\n", Simulator::Now().GetSeconds());
    backtrace_symbols_fd(bt, n, STDERR_FILENO);
    _exit(139);
}

} // namespace

int
main(int argc, char* argv[])
{
    signal(SIGSEGV, SegvHandler);

    uint32_t nx = 5;
    uint32_t ny = 5;
    double spacing = 50.0;
    double txPower = 22.0; // dBm; Spectrum PHY needs ~6 dB more than Yans to hold 50 m at 80 MHz
    uint32_t flows = 20;
    string rate = "2Mbps";
    uint32_t seed = 1;
    bool mu = true;
    string roles = "tree"; // tree = 1 AP + 1 STA (2 radios); checker = black AP / white STA-per-neighbour
    double assocTime = 10.0;  // ring/grid associations settle slowly with many BSSs
    double trafficDur = 10.0;
    CommandLine cmd;
    cmd.AddValue("nx", "Grid width", nx);
    cmd.AddValue("ny", "Grid height", ny);
    cmd.AddValue("spacing", "Node spacing (m)", spacing);
    cmd.AddValue("txPower", "Tx power (dBm)", txPower);
    cmd.AddValue("flows", "Number of random UDP flows", flows);
    cmd.AddValue("rate", "Per-flow offered rate", rate);
    cmd.AddValue("seed", "RNG run number for the flow draw", seed);
    cmd.AddValue("mu", "Install RrMultiUserScheduler on every AP", mu);
    cmd.AddValue("roles", "Role scheme: tree | checker", roles);
    cmd.AddValue("assoc", "Seconds allowed for association before traffic", assocTime);
    cmd.AddValue("dur", "Traffic duration (s)", trafficDur);
    cmd.Parse(argc, argv);

    RngSeedManager::SetRun(seed);
    const uint32_t N = nx * ny;
    const uint32_t cx = nx / 2;
    const uint32_t cy = ny / 2;

    cout << fixed << setprecision(3);
    cout << "=== grid dual-role gate: " << nx << "x" << ny << ", " << spacing << " m, "
         << txPower << " dBm, " << flows << " random flows ===\n";

    // Each entry: (AP node, its STA members, channel index). Built per role scheme, before any
    // ns-3 object exists -- resource allocation is decided independently of routing.
    struct Bss
    {
        uint32_t ap;
        vector<uint32_t> stas;
        uint32_t chan;
    };
    vector<Bss> plan;
    auto id = [nx](uint32_t x, uint32_t y) { return y * nx + x; };
    const bool adhocMc = (roles == "adhoc-mc");
    const bool adhoc = (roles == "adhoc") || adhocMc;
    if (roles != "tree" && !adhoc)
    {
        cout << "roles must be tree | adhoc | adhoc-mc\n";
        return 1;
    }

    vector<int> parent(N, -1);
    for (uint32_t y = 0; y < ny; ++y)
    {
        for (uint32_t x = 0; x < nx; ++x)
        {
            int dx = int(cx) - int(x);
            int dy = int(cy) - int(y);
            if (dx == 0 && dy == 0)
            {
                continue;
            }
            if (abs(dx) >= abs(dy) && dx != 0)
            {
                parent[id(x, y)] = id(x + (dx > 0 ? 1 : -1), y);
            }
            else
            {
                parent[id(x, y)] = id(x, y + (dy > 0 ? 1 : -1));
            }
        }
    }
    vector<vector<uint32_t>> kids(N);
    for (uint32_t v = 0; v < N; ++v)
    {
        if (parent[v] >= 0)
        {
            kids[parent[v]].push_back(v);
        }
    }
    // A node serving nobody needs no AP radio, so only real parents become BSS owners.
    if (!adhoc)
    {
        for (uint32_t u = 0; u < N; ++u)
        {
            if (!kids[u].empty())
            {
                plan.push_back({u, kids[u], (u % nx + u / nx) % 2});
            }
        }
    }

    uint32_t nChans = adhoc ? (adhocMc ? 2 : 1) : 0;
    for (const Bss& b : plan)
    {
        nChans = max(nChans, b.chan + 1);
    }
    cout << "roles=" << roles << ", " << plan.size() << " BSSs, " << nChans << " channels\n";

    NodeContainer nodes;
    nodes.Create(N);

    MobilityHelper mob;
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    for (uint32_t y = 0; y < ny; ++y)
    {
        for (uint32_t x = 0; x < nx; ++x)
        {
            pos->Add(Vector(x * spacing, y * spacing, 0.0));
        }
    }
    mob.SetPositionAllocator(pos);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    InternetStackHelper internet;
    Ipv4StaticRoutingHelper srh;
    internet.SetRoutingHelper(srh);
    internet.Install(nodes);
    Ipv4AddressHelper addr;

    vector<Ptr<MultiModelSpectrumChannel>> chans(nChans);
    for (size_t c = 0; c < chans.size(); ++c)
    {
        chans[c] = CreateObject<MultiModelSpectrumChannel>();
        chans[c]->AddPropagationLossModel(CreateObject<LogDistancePropagationLossModel>());
        chans[c]->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    }

    map<pair<uint32_t, uint32_t>, Ipv4Address> addrOf; // (node, bss) -> address
    map<pair<uint32_t, uint32_t>, uint32_t> ifaceOf;   // (node, bss) -> Ipv4 interface index
    vector<vector<uint32_t>> members;                  // bss -> member nodes (owner first)
    vector<vector<uint32_t>> nodeChans(N);
    vector<Ptr<StaWifiMac>> staMacs;
    vector<uint32_t> staNode;

    if (adhoc)
    {
        // Baseline: exactly what the harness does today -- one radio, one channel, no AP, so no
        // MU scheduler can exist. Same Spectrum PHY and Tx power as the BSS build, so the only
        // variable between the two runs is the MAC/channel structure.
        WifiMacHelper mac;
        mac.SetType("ns3::AdhocWifiMac");
        vector<uint32_t> all;
        for (uint32_t n = 0; n < N; ++n)
        {
            all.push_back(n);
        }
        for (uint32_t c = 0; c < nChans; ++c)
        {
            SpectrumWifiPhyHelper p2;
            p2.SetChannel(chans[c]);
            p2.Set("ChannelSettings",
                   StringValue("{" + to_string(CH80[c]) + ", 80, BAND_5GHZ, 0}"));
            p2.Set("TxPowerStart", DoubleValue(txPower));
            p2.Set("TxPowerEnd", DoubleValue(txPower));
            NetDeviceContainer dev = wifi.Install(p2, mac, nodes);
            addr.SetBase(Ipv4Address(("10.4." + to_string(c + 1) + ".0").c_str()),
                         "255.255.255.0");
            Ipv4InterfaceContainer ifc = addr.Assign(dev);
            for (uint32_t n = 0; n < N; ++n)
            {
                addrOf[{n, c}] = ifc.GetAddress(n);
                ifaceOf[{n, c}] =
                    nodes.Get(n)->GetObject<Ipv4>()->GetInterfaceForDevice(dev.Get(n));
                nodeChans[n].push_back(c);
            }
            members.push_back(all);
        }
    }

    for (uint32_t k = 0; k < plan.size(); ++k)
    {
        const uint32_t u = plan[k].ap;
        const uint32_t c = plan[k].chan;

        SpectrumWifiPhyHelper phy;
        phy.SetChannel(chans[c]);
        phy.Set("ChannelSettings",
                StringValue("{" + to_string(CH80[c]) + ", 80, BAND_5GHZ, 0}"));
        phy.Set("TxPowerStart", DoubleValue(txPower));
        phy.Set("TxPowerEnd", DoubleValue(txPower));
        Ssid ssid = Ssid("grid-bss-" + to_string(u));

        WifiMacHelper macAp;
        if (mu)
        {
            macAp.SetMultiUserScheduler("ns3::RrMultiUserScheduler");
        }
        macAp.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        NetDeviceContainer dAp = wifi.Install(phy, macAp, nodes.Get(u));

        NodeContainer staNodes;
        for (uint32_t v : plan[k].stas)
        {
            staNodes.Add(nodes.Get(v));
        }
        WifiMacHelper macSta;
        macSta.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        NetDeviceContainer dSta = wifi.Install(phy, macSta, staNodes);

        addr.SetBase(Ipv4Address(("10.4." + to_string(k + 1) + ".0").c_str()), "255.255.255.0");
        NetDeviceContainer both(dAp, dSta);
        Ipv4InterfaceContainer ifc = addr.Assign(both);

        // Ask Ipv4 for the interface index -- loopback is device 0 AND interface 0, so guessing
        // it from GetNDevices() is off by one and silently segfaults on the first packet.
        Ptr<Ipv4> ip4 = nodes.Get(u)->GetObject<Ipv4>();
        addrOf[{u, k}] = ifc.GetAddress(0);
        ifaceOf[{u, k}] = ip4->GetInterfaceForDevice(dAp.Get(0));
        nodeChans[u].push_back(c);

        vector<uint32_t> m{u};
        for (uint32_t i = 0; i < plan[k].stas.size(); ++i)
        {
            const uint32_t v = plan[k].stas[i];
            addrOf[{v, k}] = ifc.GetAddress(i + 1);
            ifaceOf[{v, k}] =
                nodes.Get(v)->GetObject<Ipv4>()->GetInterfaceForDevice(dSta.Get(i));
            nodeChans[v].push_back(c);
            m.push_back(v);
            staMacs.push_back(
                DynamicCast<StaWifiMac>(DynamicCast<WifiNetDevice>(dSta.Get(i))->GetMac()));
            staNode.push_back(v);
        }
        members.push_back(m);
    }

    uint32_t violations = 0;
    for (uint32_t i = 0; i < N; ++i)
    {
        set<uint32_t> seen;
        for (uint32_t c : nodeChans[i])
        {
            if (!seen.insert(c).second)
            {
                cout << "  RULE-3 VIOLATION: node" << i << " has 2 radios on channel index " << c
                     << "\n";
                ++violations;
            }
        }
    }

    // One canonical address per node: its own AP address if it owns a BSS, else its STA address.
    vector<Ipv4Address> canon(N);
    vector<uint32_t> canonBss(N);
    if (adhoc)
    {
        for (uint32_t n = 0; n < N; ++n)
        {
            canon[n] = addrOf[{n, 0}];
            canonBss[n] = 0;
        }
    }
    for (uint32_t k = 0; k < members.size(); ++k)
    {
        for (uint32_t v : members[k])
        {
            if (v == members[k][0] || canon[v] == Ipv4Address())
            {
                canon[v] = addrOf[{v, k}];
                canonBss[v] = k;
            }
        }
    }

    // IP graph: every BSS is a clique (the AP relays STA->STA inside its own BSS). Edge cost is
    // RADIO TRANSMISSIONS, not IP hops -- an AP<->STA hop is one frame, a STA<->STA hop is two
    // because the AP repeats it. Routing on hops would happily pick the twice-as-expensive edge.
    struct Edge
    {
        uint32_t to;
        uint32_t bss;
        uint32_t cost;
    };
    vector<vector<Edge>> adj(N);
    if (adhoc)
    {
        // No AP to relay, so peers talk directly -- but only to actual grid neighbours.
        for (uint32_t y = 0; y < ny; ++y)
        {
            for (uint32_t x = 0; x < nx; ++x)
            {
                // Dijkstra runs from the destination, so the edge stored at u pointing to v is
                // the one v will send on -- tag it with v's channel.
                auto ch = [&](uint32_t a, uint32_t b) {
                    return adhocMc ? uint32_t((a + b) % 2) : 0u;
                };
                if (x + 1 < nx)
                {
                    adj[id(x, y)].push_back({id(x + 1, y), ch(x + 1, y), 1});
                    adj[id(x + 1, y)].push_back({id(x, y), ch(x, y), 1});
                }
                if (y + 1 < ny)
                {
                    adj[id(x, y)].push_back({id(x, y + 1), ch(x, y + 1), 1});
                    adj[id(x, y + 1)].push_back({id(x, y), ch(x, y), 1});
                }
            }
        }
    }
    else
    {
        for (uint32_t k = 0; k < members.size(); ++k)
        {
            const auto& m = members[k];
            for (size_t i = 0; i < m.size(); ++i)
            {
                for (size_t j = i + 1; j < m.size(); ++j)
                {
                    const uint32_t w = (i == 0) ? 1 : 2; // m[0] is the AP
                    adj[m[i]].push_back({m[j], k, w});
                    adj[m[j]].push_back({m[i], k, w});
                }
            }
        }
    }

    uint32_t routesInstalled = 0;
    uint64_t totalCost = 0;
    for (uint32_t d = 0; d < N; ++d)
    {
        vector<uint32_t> dist(N, UINT32_MAX);
        vector<int> par(N, -1);
        vector<int> pbss(N, -1);
        priority_queue<pair<uint32_t, uint32_t>,
                       vector<pair<uint32_t, uint32_t>>,
                       greater<>>
            pq;
        dist[d] = 0;
        pq.push({0, d});
        while (!pq.empty())
        {
            auto [c, u] = pq.top();
            pq.pop();
            if (c > dist[u])
            {
                continue;
            }
            for (const Edge& e : adj[u])
            {
                if (c + e.cost < dist[e.to])
                {
                    dist[e.to] = c + e.cost;
                    par[e.to] = u;
                    pbss[e.to] = e.bss;
                    pq.push({dist[e.to], e.to});
                }
            }
        }
        for (uint32_t n = 0; n < N; ++n)
        {
            if (n == d || par[n] < 0)
            {
                continue;
            }
            totalCost += dist[n];
            Ptr<Ipv4StaticRouting> sr = srh.GetStaticRouting(nodes.Get(n)->GetObject<Ipv4>());
            sr->AddHostRouteTo(canon[d],
                               addrOf[{uint32_t(par[n]), uint32_t(pbss[n])}],
                               ifaceOf[{n, uint32_t(pbss[n])}]);
            ++routesInstalled;
        }
    }
    cout << "avg radio transmissions per all-pairs flow: "
         << double(totalCost) / routesInstalled << "\n";

    // Own RNG, not ns-3's: the two builds create different numbers of Wi-Fi objects and would
    // otherwise consume the global stream differently, giving each mode different flow pairs.
    mt19937 rng(seed);
    uniform_int_distribution<uint32_t> pick(0, N - 1);
    vector<pair<uint32_t, uint32_t>> pairs;
    for (uint32_t f = 0; f < flows; ++f)
    {
        uint32_t s = pick(rng);
        uint32_t d = pick(rng);
        while (d == s)
        {
            d = pick(rng);
        }
        pairs.push_back({s, d});

        uint16_t port = PORT_BASE + f;
        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(canon[d], port));
        onoff.SetConstantRate(DataRate(rate), 1000);
        ApplicationContainer a = onoff.Install(nodes.Get(s));
        a.Start(Seconds(assocTime + 0.01 * f));
        a.Stop(Seconds(assocTime + trafficDur));

        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sk = sink.Install(nodes.Get(d));
        sk.Start(Seconds(1.0));
        sk.Stop(Seconds(assocTime + trafficDur + 1.0));
    }

    cout << members.size() << " BSSs, " << staMacs.size() << " STA radios, " << routesInstalled
         << " host routes\n";

    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();
    Simulator::Stop(Seconds(assocTime + trafficDur + 2.0));
    Simulator::Run();

    uint32_t assoc = 0;
    for (size_t i = 0; i < staMacs.size(); ++i)
    {
        if (staMacs[i]->IsAssociated())
        {
            ++assoc;
        }
        else
        {
            cout << "  node" << staNode[i] << " NOT associated\n";
        }
    }

    map<uint16_t, pair<uint64_t, uint64_t>> perPort; // port -> (tx, rx)
    uint64_t tx = 0, rx = 0, rxBytes = 0;
    double delaySum = 0.0;
    Ptr<Ipv4FlowClassifier> cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    for (const auto& [fid, st] : fm->GetFlowStats())
    {
        uint16_t p = cl->FindFlow(fid).destinationPort;
        if (p < PORT_BASE || p >= PORT_BASE + flows)
        {
            continue;
        }
        perPort[p].first += st.txPackets;
        perPort[p].second += st.rxPackets;
        tx += st.txPackets;
        rx += st.rxPackets;
        rxBytes += st.rxBytes;
        delaySum += st.delaySum.GetSeconds();
    }
    uint32_t delivered = 0;
    for (uint32_t f = 0; f < flows; ++f)
    {
        auto it = perPort.find(PORT_BASE + f);
        bool ok = (it != perPort.end() && it->second.second > 0);
        delivered += ok ? 1 : 0;
        if (!ok)
        {
            cout << "  flow " << f << " (" << pairs[f].first << " -> " << pairs[f].second
                 << ") delivered nothing\n";
        }
    }

    cout << "\naggregate over " << flows << " flows: tx=" << tx << " rx=" << rx
         << " bytes=" << rxBytes << " goodput=" << (rxBytes * 8.0 / trafficDur / 1e6)
         << " Mb/s  delay=" << (rx ? delaySum / rx * 1e3 : 0.0)
         << " ms  loss=" << (tx ? 100.0 * (tx - rx) / tx : 0.0) << " %\n";

    cout << "\n--- verdict ---\n";
    cout << "simulation completed without crashing: PASS\n";
    cout << "two channels suffice (no node with 2 radios on one channel): "
         << (violations == 0 ? "PASS" : "FAIL") << "  (" << violations << " violations)\n";
    cout << "every STA associated: " << (assoc == staMacs.size() ? "PASS" : "FAIL") << "  ("
         << assoc << "/" << staMacs.size() << ")\n";
    cout << "every random flow delivered: " << (delivered == flows ? "PASS" : "FAIL") << "  ("
         << delivered << "/" << flows << ")\n";

    Simulator::Destroy();
    return (violations == 0 && assoc == staMacs.size() && delivered == flows) ? 0 : 1;
}
