#include "applications.h"
#include "controller.h"
#include "internet.h"
#include "loss-ledger.h"

#include "ns3/rng-seed-manager.h"
#include "run-mode.h"
#include "topology.h"
#include "traffic-control.h"
#include "warmup.h"

#include "../nsga/graph.h"
#include "../nsga/nsga2.h"
#include "../nsga/routing.h" // BuildRoutes, nsga::Route
#include "../nsga/seng.h"   // SengRun: the scalar weighted-sum baseline
#include "../nsga/slice.h"   // nsga::Slice (shared GA + harness slice)
#include "../ofdma/enforcement.h" // BottleneckCapacity, ShapedRates (OFDMA source shaping)

#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"

#include <algorithm>
#include <array>
#include <numeric>
#include <random>

using namespace ns3;
using namespace std;

// Thesis harness: build the ax mesh, then run ONE mode —
//   --warmup link-measurement sweep (feeds the GA) | --sliceTest slice workload
//   --olsr OLSR baseline slice test | --gaRoute warm-up->NSGA-II->inject->run slice traffic
// A slice (nsga::Slice) is shared with the GA: members + demand drive routing, tos + port
// drive the traffic. Slice traffic install + per-slice result print live in applications.{cc,h}.
// Mode guard is in run-mode.{cc,h}; design: PLAN.md "Harness run modes".
int
main(int argc, char* argv[])
{
    // --- Topology & output ---
    uint32_t gridWidth = 10; // square grid side; 10 -> 10x10 = 100 nodes, per proposal
    double spacing = 50.0;   // distance between adjacent nodes (m)
    uint32_t chWidth = 80;   // channel width (MHz)
    string resultsDir = "scratch/mesh-slicing/results/"; // per-run output (flowmon/linktable)
    string linkTable;                                    // reuse a measured table instead of warming up
    uint32_t pktSize = 700;                              // UDP payload; proposal fixes 700 for every slice
    bool pcap = false; // OFF by default: nothing reads the pcaps and parallel runs writing
                       // them together saturated the WSL2 disk (27 GB in one sweep)

    // --- Run mode (pick one; see PLAN.md "Harness run modes") ---
    bool warmup = false;    // per-link measurement sweep (feeds the GA)
    bool sliceTest = false; // per-slice traffic test workload
    bool olsr = false;      // OLSR baseline: OLSR routes the slice traffic (no GA)
    bool gaRoute = false;   // NSGA-II: warm-up -> GA -> inject -> run slice traffic

    // --- Warm-up sweep ---
    double warmupStart = 2.0; // sim time to begin the sweep / slice traffic
    double windowSec = 0.5;   // duration of each L and R measurement window
    double primeSec = 0.1;    // per-edge pre-roll (ARP/queue settle); 0 = skip
    string satRate = "120Mb/s"; // R-window offered rate; 300 overshoots and corrupts it (warmup.h)
    bool traceR = false;        // per-edge tx/rx/loss/span in the warm-up R window
    bool flushSender = true;    // warm-up: reset the sender between links
    bool bidir = false;         // slice traffic: uplink only, as every source paper does
    double drainSec = 120.0;    // SAFETY CAP on the drain, not a drain time: the run ends as
                                // soon as nothing is held (ScheduleDrain), and is invalid if this
                                // cap is reached with packets still in the queues
    // Independent replication index. --gaSeed fixes the PROBLEM (members, pairs, evolution);
    // --rngRun varies only ns-3's own randomness, so repeats at one gaSeed measure run variance
    // and spread across gaSeeds measures model error. Murugeswari repeats 20 times and plots
    // mean with standard deviation; Deb's NSGA-II paper reports ten runs; the Smart Resource
    // Allocation paper averages ten. One run per point is below all of them.
    uint32_t rngRun = 1;
    string router = "nsga"; // nsga | seng -- which optimiser installs the routes
    string obj1 = "mean";   // mean | max -- obj[1] over URLLC pairs (max is the old form)
    double maxPerHopDelay = 120.0; // FlowMonitor loss window (s); default 10 is below our queueing
    bool perfectArp = true;          // resolve addresses before the trial, as RFC 2544 asks
    string plainQdisc = "ns3::FqCoDelQueueDisc"; // queue disc when --sliceQdisc is off (Linux default)
    double guardSec = 0.5;      // drain time between warm-up windows
    int maxRounds = 3;        // retry rounds for edges that lost every packet

    // --- Slices (shared: members + demand drive the GA; tos + port drive the traffic) ---
    uint32_t Me = 10, Mu = 10, Mc = 10;                          // slice member counts
    double embbDemand = 4.0, urllcDemand = 1.0, mmtcDemand = 2.0; // per-slice demand Mb/s
    uint32_t gaPop = 100, gaGens = 100, gaSeed = 1;              // population / generations / seed
    double gaC = 4.53;                                           // calibrated (700 B); recalibrate if pkt size changes

    // --- OFDMA enforcement (gates/ofdma-enforce-gate.cc landed; --gaRoute only) ---
    bool enforce = false;    // cap each slice source at its allocated RU share
    bool sliceQdisc = false; // enforce at L2.5 instead: per-slice Tbf on every radio
    // 1/(1 - T0/T*) with T0 = 3.9 ms measured and T* = 10 ms cited; see Alloc.md 0.6.
    double urllcHeadroom = 1.64; // URLLC gets this multiple of its demand -> lane runs at rho* = 0.61
    double embbFloor = 0.3;      // the design's one policy number; sweep it (Alloc.md 4c.7)
    double mmtcHeadroom = 1.0;   // constant-rate source cannot overshoot its own rate (Alloc.md 4c.5)
    uint32_t urllcQueuePkts = 24; // URLLC bucket depth in packets; from PLR 1e-5 at rho* (Alloc.md 4c.3b)

    // --- Per-node capacity controller (--sliceQdisc only; see harness/controller.h) ---
    double aiStep = 1.0, mdFactor = 0.5; // TCP's AIMD constants, borrowed as a control law
    uint32_t radioTarget = 10;           // radio backlog counted as congested (packets)
    double ctrlPeriod = 0.5;             // tick interval (s)
    double capMax = 80.0;                // physical ceiling on C, NOT a capacity estimate
    bool ctrlVerbose = false;
    double reuse = 1.0;   // simultaneous transmitters the grid supports (1 = one collision domain)

    CommandLine cmd;
    // Topology & output
    cmd.AddValue("gridWidth", "Square grid side (gridWidth x gridWidth nodes)", gridWidth);
    cmd.AddValue("spacing", "Distance between adjacent nodes (m)", spacing);
    cmd.AddValue("chWidth", "Channel width MHz (e.g. 20, 40, 80)", chWidth);
    cmd.AddValue("resultsDir", "Output dir for flowmon/linktable (give each parallel run its own)", resultsDir);
    cmd.AddValue("pktSize",
                 "UDP payload bytes for warm-up probes and slice traffic. The proposal fixes "
                 "700; the source papers use 1128 and 1472, and a bigger payload amortises the "
                 "per-frame cost, so this changes measured capacity",
                 pktSize);
    cmd.AddValue("linkTable",
                 "Reuse this linktable.csv instead of measuring one. Two runs that share a "
                 "table get the same graph and so the same GA routes, which is the only way "
                 "an A/B on anything downstream of routing means something",
                 linkTable);
    cmd.AddValue("pcap", "Capture per-radio pcap (off by default; huge, and parallel runs will thrash the disk)", pcap);
    // Run mode
    cmd.AddValue("warmup", "Run the staggered per-link warm-up sweep", warmup);
    cmd.AddValue("sliceTest", "Run the per-slice traffic test", sliceTest);
    cmd.AddValue("olsr", "OLSR baseline test case: sim -> OLSR -> slice traffic -> measure (no GA; warm-up stays static)", olsr);
    cmd.AddValue("gaRoute", "Warm-up -> NSGA-II -> inject the GA's static routes -> run slice traffic", gaRoute);
    // Warm-up sweep
    cmd.AddValue("warmupStart", "Sim time (s) to begin the warm-up sweep / slice traffic", warmupStart);
    cmd.AddValue("windowSec", "Duration (s) of each warm-up measurement window", windowSec);
    cmd.AddValue("primeSec", "Unmeasured pre-roll (s) per edge (ARP/queue settle); 0 = skip", primeSec);
    cmd.AddValue("satRate", "Warm-up R-window offered rate (must exceed the link ceiling)", satRate);
    cmd.AddValue("traceR", "Print per-edge tx/rx/loss/span in the warm-up R window", traceR);
    cmd.AddValue("guardSec", "Warm-up drain time between measurement windows (s)", guardSec);
    cmd.AddValue("maxRounds", "Warm-up retry rounds", maxRounds);
    cmd.AddValue("flushSender", "Warm-up: drop whatever the sender still holds between links", flushSender);
    cmd.AddValue("bidir", "Slice traffic both ways (default uplink only, as in every source paper)", bidir);
    cmd.AddValue("drainSec", "Safety cap on the drain (s); the run ends when the queues are empty", drainSec);
    cmd.AddValue("rngRun", "ns-3 replication index: same problem, different randomness", rngRun);
    cmd.AddValue("router", "Route optimiser: nsga (Pareto) or seng (scalar weighted sum)", router);
    cmd.AddValue("obj1", "URLLC objective over pairs: mean (default) or max (the old form)", obj1);
    cmd.AddValue("maxPerHopDelay", "FlowMonitor: seconds unseen before a packet counts as lost", maxPerHopDelay);
    cmd.AddValue("perfectArp", "Pre-resolve every ARP entry before traffic (RFC 2544 learning frames)", perfectArp);
    cmd.AddValue("plainQdisc", "Queue disc when --sliceQdisc is off (default = Linux's fq_codel; ns3::FifoQueueDisc for the dumb-buffer arm)", plainQdisc);
    // Slices
    cmd.AddValue("Me", "eMBB member count", Me);
    cmd.AddValue("Mu", "URLLC member count", Mu);
    cmd.AddValue("Mc", "mMTC member count", Mc);
    cmd.AddValue("embbDemand", "eMBB demand Mb/s (GA + traffic)", embbDemand);
    cmd.AddValue("urllcDemand", "URLLC demand Mb/s (GA + traffic)", urllcDemand);
    cmd.AddValue("mmtcDemand", "mMTC demand Mb/s (GA + traffic)", mmtcDemand);
    cmd.AddValue("gaPop", "GA population size", gaPop);
    cmd.AddValue("gaGens", "GA generations", gaGens);
    cmd.AddValue("gaSeed", "GA RNG seed (members + evolution)", gaSeed);
    cmd.AddValue("gaC", "GA calibrated delay constant ms", gaC);
    cmd.AddValue("enforce", "Shape each slice source to its OFDMA RU share (--gaRoute only)", enforce);
    cmd.AddValue("sliceQdisc",
                 "Enforce the RU shares at L2.5 (per-slice Tbf on every radio) instead of at "
                 "the source; sources then offer their full demand (--gaRoute only)",
                 sliceQdisc);
    cmd.AddValue("urllcHeadroom",
                 "URLLC share = demand x this (>=1; 1.64 -> lane runs 61% busy, from T0 and T*)",
                 urllcHeadroom);
    cmd.AddValue("embbFloor", "eMBB's guaranteed share of what URLLC leaves (policy; sweep it)",
                 embbFloor);
    cmd.AddValue("mmtcHeadroom", "mMTC share = demand x this (1.0 = no headroom)", mmtcHeadroom);
    cmd.AddValue("urllcQueuePkts", "URLLC bucket depth in packets (default 24, from the cited PLR target)", urllcQueuePkts);
    cmd.AddValue("aiStep", "Controller additive increase per tick (Mb/s)", aiStep);
    cmd.AddValue("mdFactor", "Controller multiplicative decrease", mdFactor);
    cmd.AddValue("radioTarget", "Radio backlog counted as congested (packets)", radioTarget);
    cmd.AddValue("ctrlPeriod", "Controller tick interval (s)", ctrlPeriod);
    cmd.AddValue("capMax", "Physical ceiling on per-node C (Mb/s)", capMax);
    cmd.AddValue("ctrlVerbose", "Print the controller's per-tick capacity spread", ctrlVerbose);
    cmd.AddValue("reuse", "Spatial-reuse factor: how many parts of the grid transmit at once", reuse);
    cmd.Parse(argc, argv);

    // One routing method per run (guard in run-mode.{cc,h}).
    if (!ResolveRunMode(olsr, gaRoute, warmup, sliceTest))
    {
        return 1;
    }

    PrepareResultsDir(resultsDir);

    // MUST precede BuildGrid..., not merely precede the traffic: every random variable a node
    // owns -- above all the MAC's backoff -- is assigned its stream when the device is created,
    // and SetRun only affects streams assigned after it. Called too late (it was, until
    // 2026-10-11) the radios all draw run 1 whatever --rngRun says, and replications come back
    // bit-identical: rngRun 1, 2 and 7 gave the same URLLC delay to every digit.
    RngSeedManager::SetRun(rngRun);

    const uint32_t numNodes = gridWidth * gridWidth;
    MeshTopology mesh =
        BuildGridAdhocAxTopology(gridWidth, gridWidth, spacing, chWidth,
                                 pcap ? resultsDir + "mesh" : "");
    // --enforce and --sliceQdisc are the two arms of the same experiment: --enforce lowers the
    // offered load, --sliceQdisc leaves it alone and polices the wire. Running both at once
    // measures neither.
    if (enforce && sliceQdisc)
    {
        cout << "error: --enforce and --sliceQdisc are alternatives, pick one\n";
        return 1;
    }
    SetUrllcQueuePackets(urllcQueuePkts); // before InstallSliceQdiscs: the band is sized at install
    QueueDiscContainer sliceQdiscs;
    Ipv4InterfaceContainer interfaces =
        InstallInternetStack(mesh, olsr, sliceQdisc ? &sliceQdiscs : nullptr, plainQdisc);
    if (perfectArp)
    {
        PerfectArp(mesh, interfaces);
    }

    // Lives to the end of main: the controller reschedules itself for the whole traffic phase.
    SliceController ctrl;
    SliceController::Config ctrlCfg;
    ctrlCfg.urllcHeadroom = urllcHeadroom;
    ctrlCfg.embbFloor = embbFloor;
    ctrlCfg.mmtcHeadroom = mmtcHeadroom;
    array<uint64_t, nsga::NUM_SLICES> srcPkts = {};
    ctrlCfg.aiStep = aiStep;
    ctrlCfg.mdFactor = mdFactor;
    ctrlCfg.radioTargetPkts = radioTarget;
    ctrlCfg.period = ctrlPeriod;
    ctrlCfg.capMax = capMax;
    ctrlCfg.verbose = ctrlVerbose;

    InstallLossLedger(mesh); // name every frame the medium destroys; see loss-ledger.h
    FlowMonitorHelper flowmonHelper;
    // FlowMonitor declares a packet lost once it has gone MaxPerHopDelay unseen (default 10 s),
    // erases its tracking, and then IGNORES it if it does arrive (ReportLastRx returns early on
    // an untracked packet). mMTC's bucket alone can hold ~6 s at its cap, so the default turned
    // late deliveries into phantom loss AND dropped them out of the delay average. Raise it well
    // past any queueing delay the harness can produce; it is a measurement window, not a timeout.
    flowmonHelper.SetMonitorAttribute("MaxPerHopDelay", TimeValue(Seconds(maxPerHopDelay)));
    Ptr<FlowMonitor> flowMonitor = flowmonHelper.InstallAll();

    // Build the 3 slices once: the shared instance the GA optimizes and the traffic exercises
    // (same Slice code path for any member count). Two member layouts:
    //   --gaRoute        : disjoint RANDOM member sets (Me/Mu/Mc) -> the full multi-member test
    //   --sliceTest/olsr : the classic single-pair 0<->2 contention test (each slice = {0,2})
    // eMBB >> URLLC >> mMTC ports so results group by range.
    mt19937 rng(gaSeed);
    array<nsga::Slice, nsga::NUM_SLICES> slices;
    const bool needSlices = gaRoute || sliceTest;
    vector<uint32_t> em, um, mm;
    if (gaRoute)
    {
        if (Me + Mu + Mc > numNodes)
        {
            cout << "error: Me+Mu+Mc (" << (Me + Mu + Mc) << ") exceeds nodes (" << numNodes << ")\n";
            return 1;
        }
        // Members are consumed two at a time as source->destination pairs (Slice::Pairs), so an
        // odd count would leave a member with nobody to talk to.
        if (Me % 2 || Mu % 2 || Mc % 2)
        {
            cout << "error: Me/Mu/Mc must be even - members are drawn as src->dst pairs\n";
            return 1;
        }
        vector<uint32_t> pool(numNodes);
        iota(pool.begin(), pool.end(), 0);
        shuffle(pool.begin(), pool.end(), rng);
        em = {pool.begin(), pool.begin() + Me};
        um = {pool.begin() + Me, pool.begin() + Me + Mu};
        mm = {pool.begin() + Me + Mu, pool.begin() + Me + Mu + Mc};
    }
    else if (sliceTest)
    {
        em = um = mm = {0, 2}; // classic contention test: one pair, 0 -> 2
    }
    if (needSlices)
    {
        slices[0] = {.name = "eMBB", .members = em, .demandMbps = embbDemand, .tos = SliceTos::Embb, .port = 10000};
        slices[1] = {.name = "URLLC", .members = um, .demandMbps = urllcDemand, .tos = SliceTos::Urllc, .port = 20000};
        slices[2] = {.name = "mMTC", .members = mm, .demandMbps = mmtcDemand, .tos = SliceTos::Mmtc, .port = 30000};
    }

    // Warm-up session (drives its own edge sweep + retry stop). Used by pure --warmup and
    // by --gaRoute's measurement phase.
    WarmupSession session;
    auto scheduleWarmup = [&]() {
        session.mesh = mesh;
        session.interfaces = interfaces;
        session.classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
        session.monitor = flowMonitor;
        session.windowSec = windowSec;
        session.primeSec = primeSec;
        session.satRate = satRate;
        session.traceR = traceR;
        session.guardSec = guardSec;
        session.pktSize = pktSize;
        session.maxRounds = maxRounds;
        session.flushSender = flushSender;
        Simulator::Schedule(Seconds(warmupStart), &BeginWarmup, &session);
    };
    auto writeLinkTable = [&]() {
        vector<LinkMeasurement> table =
            ExtractLinkTable(session.ctx, session.classifier, flowMonitor);
        MedianCorrectOutliers(table);
        WriteLinkTable(table, resultsDir + "linktable.csv");
    };

    const double trafficDur = 8.0; // slice-traffic window (s)

    if (gaRoute)
    {
        using namespace nsga;
        // Phase 1: measured link table. Warm-up is a radio measurement, so it lands on
        // slightly different numbers every run and the GA then picks different routes --
        // which makes any A/B downstream of routing meaningless. --linkTable reuses one
        // table so the compared runs share a graph, and skips the sweep's ~100 s.
        string tablePath = linkTable.empty() ? resultsDir + "linktable.csv" : linkTable;
        if (linkTable.empty())
        {
            scheduleWarmup();
            Simulator::Run();
            writeLinkTable();
        }
        else
        {
            cout << "reusing link table " << tablePath << " (no warm-up sweep)\n";
        }

        // GA on the measured table -> knee solution -> install its routes.
        nsga::g_obj1Max = (obj1 == "max");
        Graph g = Graph::FromLinkTable(tablePath, numNodes);
        Params params{gaPop, gaGens, 0.1, gaC};
        Individual pick;
        bool havePick = false;
        if (router == "seng")
        {
            // Same graph, same slices, same population size and generations as NSGA-II: the only
            // difference is that one number is minimised instead of a front being sorted.
            pick = SengRun(g, slices, params, rng);
            havePick = true;
        }
        else
        {
            vector<Individual> front = Run(g, slices, params, rng);
            if (front.empty())
            {
                cout << "gaRoute error: GA returned an empty Pareto front\n";
            }
            else
            {
                pick = front[KneePoint(front)];
                havePick = true;
            }
        }
        if (havePick)
        {
            vector<Route> routes = BuildRoutes(g, pick, slices);
            InjectStaticRoutes(mesh, interfaces, routes);
            cout << "\n=== " << (router == "seng" ? "Seng scalar" : "NSGA-II knee")
                 << " routes injected into harness ===\n";
            cout << "pick: eMBB_tp=" << pick.obj[0] << " Mb/s  URLLC_delay=" << pick.obj[1]
                 << " ms  mMTC_links=" << static_cast<int>(pick.obj[2])
                 << "  stranded=" << pick.stranded << "\n";
            // Stranded members mean no feasible solution was found at all, and every objective
            // is then measured over whoever happened to stay connected.
            if (pick.stranded > 0)
            {
                cout << "WARNING: no fully connected solution found; " << pick.stranded
                     << " member(s) have no route. Objectives below are not comparable.\n";
            }
            cout << "installed " << routes.size() << " static host routes\n";

            // Phase 2: run the slice traffic on the injected routes (times are delay-from-now
            // since the sim clock already advanced through warm-up). With --enforce each source
            // is capped at its RU share, so the modeled OFDMA split is real on the wire.
            array<double, nsga::NUM_SLICES> onWire;
            for (int i = 0; i < nsga::NUM_SLICES; ++i)
            {
                onWire[i] = slices[i].demandMbps;
            }
            if (sliceQdisc)
            {
                // Capacity is MEASURED, per node, for the whole run -- never computed from a
                // model. EffectiveCapacity (link R / hops) used to supply it and was ~7x out
                // (28.6 ms predicted against 168 measured), so the loop replaces it rather than
                // repairing it. Sources stay at full demand; the buckets do the policing.
                ctrl.Start(mesh.devices,
                           sliceQdiscs,
                           {slices[0].demandMbps, slices[1].demandMbps, slices[2].demandMbps},
                           ctrlCfg,
                           1.0 + ctrlCfg.period, // one period after the traffic starts
                           trafficDur);
                cout << "per-node capacity control ON (AIMD +" << ctrlCfg.aiStep << "/x"
                     << ctrlCfg.mdFactor << " on radio queue > " << ctrlCfg.radioTargetPkts
                     << " pkt, every " << ctrlCfg.period << "s; Headroom=" << urllcHeadroom
                     << ", F=" << embbFloor << ", mmtcHeadroom=" << mmtcHeadroom << ")\n";
            }
            else if (enforce)
            {
                // Source-cap arm, kept as the baseline it was demoted to: it cuts offered load
                // AND polices at once, so its numbers cannot separate the two. Still on
                // EffectiveCapacity, which is why it is a baseline and not the design.
                double R = nsga::ofdma::EffectiveCapacity(g, pick, slices, routes, reuse);
                if (R <= 0.0)
                {
                    cout << "enforce: no measured link on the chosen routes; running unshaped\n";
                }
                else
                {
                    onWire = nsga::ofdma::ShapedRates(R, slices, urllcHeadroom, embbFloor,
                                                      mmtcHeadroom);
                    cout << "OFDMA enforcement ON (source cap, link R="
                         << nsga::ofdma::BottleneckCapacity(g, routes) << " Mb/s over "
                         << nsga::ofdma::AverageHops(g, pick, slices)
                         << " hops, reuse=" << reuse << " -> effective R=" << R
                         << " Mb/s, urllcHeadroom=" << urllcHeadroom << ")\n";
                    for (int i = 0; i < nsga::NUM_SLICES; ++i)
                    {
                        cout << "  " << slices[i].name << ": demand=" << slices[i].demandMbps
                             << " -> onWire=" << onWire[i] << " Mb/s\n";
                    }
                }
            }
            for (int i = 0; i < nsga::NUM_SLICES; ++i)
            {
                InstallSliceTraffic(mesh, interfaces, slices[i], onWire[i], bidir, 1.0, 1.0 + trafficDur,
                                    pktSize);
            }
            // The drain is a condition, not a constant: poll until the queues are empty, and
            // treat drainSec as the safety cap. Stop() is still armed at the cap in case the
            // poller never fires.
            ScheduleDrain(mesh, 1.0 + trafficDur, 0.05, drainSec);
            Simulator::Stop(Seconds(1.0 + trafficDur + drainSec));
            Simulator::Run();
            cout << "sim ended at t=" << Simulator::Now().GetSeconds() << "s\n";
            srcPkts = PrintSliceResults(slices, flowMonitor,
                              DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier()));
            if (sliceQdisc)
            {
                PrintSliceQdiscStats(sliceQdiscs, srcPkts);
                PrintCapVsOffered(sliceQdiscs, trafficDur);
            }
            PrintLossLedger();
            PrintHeldPackets(mesh);
            PrintDrain(mesh, drainSec);
        }
    }
    else if (sliceTest) // includes --olsr (routes via OLSR); plain --sliceTest is static/direct-only
    {
        for (const nsga::Slice& s : slices)
        {
            InstallSliceTraffic(mesh, interfaces, s, s.demandMbps, bidir, warmupStart,
                                warmupStart + trafficDur);
        }
        Simulator::Stop(Seconds(warmupStart + trafficDur + 2.0));
        Simulator::Run();
        PrintSliceResults(slices, flowMonitor,
                          DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier()));
    }
    else // pure --warmup
    {
        scheduleWarmup();
        Simulator::Run();
        writeLinkTable();
    }

    flowMonitor->SerializeToXmlFile(resultsDir + "flowmon.xml", true, true);
    Simulator::Destroy();
    return 0;
}
