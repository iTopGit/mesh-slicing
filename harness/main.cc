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
#include "../ofdma/allocator.h" // ofdma::Allocate
#include "../nsga/nsga2.h"
#include "../nsga/routing.h" // BuildRoutes, nsga::Route
#include "../nsga/seng.h"   // SengRun: the scalar weighted-sum baseline
#include "../nsga/slice.h"   // nsga::Slice (shared GA + harness slice)

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
    string satRate = "120Mb/s"; // R-window offered rate; 300 overshoots and corrupts it (warmup.h)
    bool traceR = false;        // per-edge tx/rx/loss/span in the warm-up R window
    bool flushSender = true;    // warm-up: reset the sender between links
    bool bidir = false;         // slice traffic: uplink only, as every source paper does
    // Total time the sources send. A BUDGET, not the measured window: the first part of it is the
    // initial transient and is deliberately thrown away. It only has to be long enough that the
    // queues reach equilibrium before the window opens, and the occupancy line REPORTS it rather than
    // assuming it -- a run whose queues were still filling is reported as not-settled.
    double trafficSec = 40.0;
    // The measured window: the LAST `measureSec` of the traffic budget. Traffic stop stays fixed
    // because an ns-3 application's stop event is scheduled at initialisation and cannot be moved
    // mid-run, so the window is placed at the end rather than triggered by the settle condition.
    //
    // DECLARED, not derived: window length sets statistical precision, not correctness, and no
    // physical quantity picks a number here. It gets the treatment `stallSec` got -- fixed, then
    // shown not to matter by measuring at 5 / 10 / 20 s. Claiming a derivation for it would be the
    // `urllcHeadroom = 1.64` mistake again.
    // Comma-separated so several NESTED windows can be measured in ONE run: they all end when the
    // sources stop, so "2,10,20" gives [39,41], [31,41], [21,41]. Same run, same randomness, so a
    // difference between them is the window LENGTH and nothing else -- which is a far cleaner
    // insensitivity test than comparing separate runs, and costs one run instead of three.
    string measureSec = "10";
    // Coarse TRIPWIRE, not a precision test: a window ending more than this many times fuller than
    // it started contained net accumulation, so its figures are a transient. 2x is declared, and
    // the point of a tripwire is that the exact value does not matter -- a 40 s run that cannot
    // settle indicates a defect in the code or the configuration, not a window too short
    // (user, 2026-10-15). Replaces a 2-run up-front check of --trafficSec: every run now gates
    // itself, so nothing is spent unless something actually trips.
    double settleGrowth = 2.0;
    // 0 = derive as CODEL_TARGET_SEC x flows-per-slice (see SetSliceLatencyTargets). An explicit
    // value overrides it, which is how the derivation gets swept rather than asserted.
    double cobaltTargetMs = 0.0;

    // WALL-CLOCK SAFETY ONLY, never a verdict input (raised from 120 on 2026-10-14, user).
    // The run ends on MOVEMENT: DrainPoll stops as soon as nothing is held, or as soon as the held
    // count has sat unchanged for --stallSec, and PrintDrain judges the cause. A timer was casting
    // the deciding vote on a question the movement test had already answered -- 5 runs were
    // discarded as INVALID while their backlog was still visibly SHRINKING, and discards track
    // overload, so that silently deleted the worst runs from both arms.
    //
    // DERIVED, not picked: the slowest a bucket can legally empty is one full depth at the
    // enforced minimum share. ns-3's default depth is 1000 packets, on-wire 728 B, and
    // traffic-control.cc floors every share at one MTU per control period (0.024 Mb/s at the
    // 0.5 s default) -- so 1000 x 728 x 8 / 0.024e6 = 243 s. Rounded up to 300 for the pipeline
    // behind it. A healthy run never reaches this: the stall test fires 2 s after progress stops.
    double drainSec = 300.0;
    // Sweep the MAC queues during the drain. On by default because a wedged node is a defect, but
    // exposed so the sweep can be removed and the run re-measured -- it reaches into ns-3's MAC,
    // which is a simulator argument rather than a networking one, so it must earn its place.
    bool drainReap = true;
    double stallSec = 2.0;      // how long the held count may sit unchanged before the queues are
                                // declared not-draining; exposed only so the verdict's
                                // insensitivity to it can be shown (loss-ledger.cc)
    // Independent replication index. --gaSeed fixes the PROBLEM (members, pairs, evolution);
    // --rngRun varies only ns-3's own randomness, so repeats at one gaSeed measure run variance
    // and spread across gaSeeds measures model error. Murugeswari repeats 20 times and plots
    // mean with standard deviation; Deb's NSGA-II paper reports ten runs; the Smart Resource
    // Allocation paper averages ten. One run per point is below all of them.
    uint32_t rngRun = 1;
    string router = "nsga"; // nsga | seng -- which optimiser installs the routes
    string obj1 = "mean";   // mean | max -- obj[1] over URLLC pairs (max is the old form)
    string obj0 = "spare";  // spare | alloc -- obj[0] form (alloc is the degenerate old one)
    // 0 = derive as the whole simulated span, so NOTHING is written off as lost while the run is
    // still going. It has to track drainSec: ns-3's 10 s default wrote off 1029 mMTC packets and
    // shifted reported loss 9.3 points, and a fixed 120 against a 300 s drain would do it again.
    double maxPerHopDelay = 0.0;
    bool perfectArp = true;          // resolve addresses before the trial, as RFC 2544 asks
    string plainQdisc = "ns3::FqCoDelQueueDisc"; // queue disc when --sliceQdisc is off (Linux default)
    string sliceAqm = "ns3::FifoQueueDisc";      // the Tbf's child; peek-safe discs only. Fifo is
                                                 // the default from 2026-10-16: Cobalt measured
                                                 // identical on URLLC and eMBB delay at every B,
                                                 // helped only a squeezed mMTC, and paid loss for
                                                 // it. It existed to repair AIMD damage.
    double guardSec = 0.5;      // drain time between warm-up windows
    int maxRounds = 3;        // retry rounds for edges that lost every packet
    uint32_t sampleEdges = 0; // 0 = sweep every edge; see WarmupSession::sampleEdges

    // --- Slices (shared: members + demand drive the GA; tos + port drive the traffic) ---
    // 0 on all three = derive from the grid (see the member draw below). Setting any of them
    // switches off the derivation entirely, so an unset slice is then genuinely empty, which is
    // how a two-slice arm is expressed.
    uint32_t Me = 0, Mu = 0, Mc = 0;                             // slice member counts
    double embbDemand = 4.0, urllcDemand = 1.0, mmtcDemand = 2.0; // per-slice demand Mb/s
    uint32_t gaPop = 100, gaGens = 100, gaSeed = 1;              // population / generations / seed
    // T0, the per-hop delay in T = T0/(1-U), and ONLY a ranking scale for obj[1] -- the caps come
    // from the clean link table instead (PLAN.md step 3). 0.989 ms is calibrated on this mesh:
    // obj[1] is exactly linear in T0 (about 6.22 x T0), so the value that makes the prediction
    // match the measurement is one division. It is a fit AT A LOAD, not a constant: holding one
    // value fixed, the prediction is 1.01x the measurement at 7 Mb/s offered, 2.99x at 14 and
    // 6.97x at 28. See PLAN.md step 2 for that table and for why no single value serves every load.
    //
    // 1.1796 is the fit on THIS grid, 10x10. It replaces 0.989, which was fitted on 5x5 at 7 Mb/s
    // and was being applied to 10x10 as an undeclared reuse -- worth 59% on H (2.935 vs 4.682 at
    // 6.667 mean hops). The earlier 4.53 came from a 2-NODE gate and was 4.6x too large here, large
    // enough to print INFEASIBLE on runs that comfortably met their target.
    //
    // Three things this value is NOT, all owed to PLAN.md step 9. It is ONE fit with no error bar.
    // It is fitted from URLLC's own measured delay and then sizes URLLC's lane, which is rule 1
    // circularity, live rather than latent. And it sits near a cliff: D = mean hops x T0 must stay
    // below T*, so at 6.667 hops any T0 >= 1.5 trips the admission limit.
    //
    // What saves the comparison is that it cannot reach it: D uses Graph::AllPairsMean of the LINK
    // TABLE, so H is identical in both arms whatever T0 is. T0 therefore threatens only absolute
    // claims ("URLLC meets 10 ms"), never an A/B -- which is why it is declared here and swept
    // rather than blocking the measurement.
    double t0Ms = 1.1796;
    // T*: URLLC's delay target. Cited, never measured -- IEEE 802.11 RTA TIG via Bankov,
    // Chemrov & Khorov, "typical RTA demand very low latency (<= 10 ms)".
    double urllcRhoStar = 0.5;   // URLLC's max share of the medium -- "no more than half"; see below
    double urllcTargetMs = 10.0;

    // --- Enforcement: per-slice Tbf on every radio, fed by the per-node capacity controller ---
    bool sliceQdisc = false;
    // H = 1/rho*, computed below from --urllcRhoStar. Not a free parameter and not derived from a
    // delay budget any more -- see the block above the assignment.
    double urllcHeadroom = 0.0;
    double embbFloor = 0.3;      // the design's one policy number; sweep it (Alloc.md 4c.7)
    // DECLARED, not derived, and deliberately so (settled 2026-10-12 with the user). A cap equal
    // to a slice's own demand puts its lane at utilization 1, where a work-conserving queue has
    // unbounded delay -- queueing theory, no slice target needed. 1.2 leaves 17% slack so the lane
    // drains. No delay or loss target is claimed for mMTC: neither source paper sets one, both call
    // it delay-insensitive, and it is out of this thesis's scope. The standing condition on a
    // declared constant applies -- no reported conclusion may depend on its value, which is
    // measured by sweeping it and showing eMBB and URLLC do not move.
    bool secondRound = false;    // two-round allocation, OFF: it erodes the burst headroom it was
                                 // meant to protect (see ofdma/allocator.cc). --secondRound=true
                                 // re-enables it as a measured arm.
    string fixedShare;           // "eMBB,URLLC,mMTC" ratios; empty = allocator runs. See controller.h
    double mmtcHeadroom = 1.2;

    // --- Per-node capacity controller (--sliceQdisc only; see harness/controller.h) ---
    double aiStep = 1.0, mdFactor = 0.5; // TCP's AIMD constants, borrowed as a control law
    uint32_t radioTarget = 10;           // radio backlog counted as congested (packets)
    double ctrlPeriod = 0.5;             // tick interval (s)
    double admitBudgetMax = 80.0;   // upper limit on the AIMD admit budget, not a measured capacity
    bool ctrlVerbose = false;

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
    cmd.AddValue("satRate", "Warm-up R-window offered rate (must exceed the link ceiling)", satRate);
    cmd.AddValue("traceR", "Print per-edge tx/rx/loss/span in the warm-up R window", traceR);
    cmd.AddValue("guardSec", "Warm-up drain time between measurement windows (s)", guardSec);
    cmd.AddValue("maxRounds", "Warm-up retry rounds", maxRounds);
    cmd.AddValue("sampleEdges",
                 "Warm-up: measure only this many edges, spread by grid position "
                 "(0 = all). For large grids; feed the result to codes/filltable.py",
                 sampleEdges);
    cmd.AddValue("flushSender", "Warm-up: drop whatever the sender still holds between links", flushSender);
    cmd.AddValue("bidir", "Slice traffic both ways (default uplink only, as in every source paper)", bidir);
    cmd.AddValue("drainSec", "Safety cap on the drain (s); the run ends when the queues are empty", drainSec);
    cmd.AddValue("cobaltTargetMs",
                 "Slice-band AQM sojourn target (ms); 0 = derive as 5 ms x flows-per-slice so a "
                 "slice queue treats each flow as the baseline's per-flow queues do",
                 cobaltTargetMs);
    cmd.AddValue("settleGrowth",
                 "Settle tripwire: a measurement window ending more than this many times fuller "
                 "than it opened is reported NOT SETTLED (its figures are a transient)",
                 settleGrowth);
    cmd.AddValue("trafficSec",
                 "Total time the sources send (s). A budget: the transient at the start is "
                 "discarded and only the last --measureSec is measured",
                 trafficSec);
    cmd.AddValue("measureSec",
                 "Measured window length(s) in seconds, taken as the LAST part of --trafficSec. "
                 "Comma-separated for several nested windows in one run, e.g. 2,10,20",
                 measureSec);
    cmd.AddValue("drainReap",
                 "Sweep expired MPDUs from the radio queues during the drain (false = let ns-3's "
                 "own lazy reap stand, which tests whether the sweep is still needed)",
                 drainReap);
    cmd.AddValue("stallSec", "Held count unchanged for this long (s) = queues declared not-draining", stallSec);
    cmd.AddValue("rngRun", "ns-3 replication index: same problem, different randomness", rngRun);
    cmd.AddValue("router", "Route optimiser: nsga (Pareto) or seng (scalar weighted sum)", router);
    cmd.AddValue("obj1", "URLLC objective over pairs: mean (default) or max (the old form)", obj1);
    cmd.AddValue("obj0", "eMBB objective: spare (default, R(1-U)) or alloc (the old fair share)", obj0);
    cmd.AddValue("maxPerHopDelay",
                 "FlowMonitor: seconds unseen before a packet counts as lost (0 = the whole "
                 "simulated span, so nothing is written off while the run is still draining)",
                 maxPerHopDelay);
    cmd.AddValue("perfectArp", "Pre-resolve every ARP entry before traffic (RFC 2544 learning frames)", perfectArp);
    cmd.AddValue("plainQdisc", "Queue disc when --sliceQdisc is off (default = Linux's fq_codel; ns3::FifoQueueDisc for the dumb-buffer arm)", plainQdisc);
    cmd.AddValue("sliceAqm",
                 "The Tbf's child discipline on the --sliceQdisc path: ns3::CobaltQueueDisc "
                 "(sojourn-bounded) or ns3::FifoQueueDisc (none). Peek-safe discs only",
                 sliceAqm);
    // Slices
    cmd.AddValue("Me", "eMBB member count (0 on all three = derive from the grid)", Me);
    cmd.AddValue("Mu", "URLLC member count (0 on all three = derive from the grid)", Mu);
    cmd.AddValue("Mc", "mMTC member count (0 on all three = derive from the grid)", Mc);
    cmd.AddValue("embbDemand", "eMBB demand Mb/s (GA + traffic)", embbDemand);
    cmd.AddValue("urllcDemand", "URLLC demand Mb/s (GA + traffic)", urllcDemand);
    cmd.AddValue("mmtcDemand", "mMTC demand Mb/s (GA + traffic)", mmtcDemand);
    cmd.AddValue("gaPop", "GA population size", gaPop);
    cmd.AddValue("gaGens", "GA generations", gaGens);
    cmd.AddValue("gaSeed", "GA RNG seed (members + evolution)", gaSeed);
    cmd.AddValue("t0", "T0: per-hop delay (ms) in T = T0/(1-U); feeds obj[1] and the headroom", t0Ms);
    cmd.AddValue("urllcRhoStar", "URLLC's max share of the medium (1/3 = fair share with 3 slices; defensible range 1/3..1/2)", urllcRhoStar);
    cmd.AddValue("urllcTarget", "T*: URLLC delay target (ms), cited from the 802.11 RTA TIG -- REPORTED against, no longer sizes the lane", urllcTargetMs);
    cmd.AddValue("sliceQdisc",
                 "Enforce the RU shares at L2.5 (per-slice Tbf on every radio) instead of at "
                 "the source; sources then offer their full demand (--gaRoute only)",
                 sliceQdisc);
    cmd.AddValue("urllcHeadroom",
                 "URLLC share = demand x this (>=1); 0 = derive from the topology and T*",
                 urllcHeadroom);
    cmd.AddValue("embbFloor", "eMBB's guaranteed share of what URLLC leaves (policy; sweep it)",
                 embbFloor);
    cmd.AddValue("mmtcHeadroom", "mMTC share = demand x this (1.0 = no headroom)", mmtcHeadroom);
    cmd.AddValue("secondRound", "Two-round allocation: reclaim unused shares and re-offer them (false = one-pass arm)", secondRound);
    cmd.AddValue("fixedShare",
                 "COMPARATOR: fixed RU ratios \"eMBB,URLLC,mMTC\" of the admit budget, demand "
                 "ignored (Marrone et al.). Empty = the allocator runs instead",
                 fixedShare);
    cmd.AddValue("aiStep", "Controller additive increase per tick (Mb/s)", aiStep);
    cmd.AddValue("mdFactor", "Controller multiplicative decrease", mdFactor);
    cmd.AddValue("radioTarget", "Radio backlog counted as congested (packets)", radioTarget);
    cmd.AddValue("ctrlPeriod", "Controller tick interval (s)", ctrlPeriod);
    cmd.AddValue("admitBudgetMax",
                 "Upper limit on the per-node ADMIT BUDGET (Mb/s) -- a bound on the AIMD search. "
                 "The budget is what a node is willing to admit, never a measurement of capacity",
                 admitBudgetMax);
    cmd.AddValue("ctrlVerbose", "Print the controller's per-tick admit-budget spread", ctrlVerbose);
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
    // Derive URLLC's lane from the topology before anything is installed: the bucket depth is
    // fixed when the band is created, so this has to happen here. It needs the link table, which
    // exists up front only when --linkTable supplies one; a fresh warm-up measures it later, so
    // that path falls back to the declared values and says so.
    nsga::Graph::PathMeans topo;
    if (!linkTable.empty())
    {
        topo = nsga::Graph::FromLinkTable(linkTable, numNodes).AllPairsMean();
        cout << "topology: " << topo.hops << " mean hops, clean-link path delay " << topo.delayMs
             << " ms" << (topo.connected ? "" : " (WARNING: mesh is partitioned)") << "\n";
    }
    // URLLC's lane is set by rho* -- the fullest its lane may run, and equally the most of the
    // medium it may claim -- and NOT by a T0-derived delay budget. Two reasons the old derivation
    // was retired on 2026-10-14, both measured:
    //
    //   circular: T0 was FITTED to URLLC's own measured delay and then used to size URLLC's lane,
    //             which is rule 1. And it is a fit AT A LOAD - the model reads 78-124x low at
    //             12 Mb/s - so no amount of careful measurement makes one value serve every load.
    //   not transferable: D = mean hops x T0 grows with the grid, so a T0 fitted at 10x10 makes
    //             16x16 and larger abort on the admission limit (D = 12.6 / 18.9 / 25.1 ms against
    //             a 10 ms target). rho* carries no hops term and means the same at every size.
    //
    // The defensible range is [1/3, 1/2]: at least URLLC's fair share with three slices, never more
    // than half the medium for one slice. The conclusion was swept inside it and does not depend on
    // the point -- URLLC is fastest and lowest-loss at both ends.
    //
    // 1/2 is the default (user, 2026-10-14) because every slice measured better there: eMBB
    // 3059 -> 3034 ms and 30.0 -> 23.6% loss, mMTC 1677 -> 1233 ms, URLLC 409 -> 273 ms at 0% loss.
    // Note WHY, because it is not "a cap costs the others nothing": rho* and H are reciprocals, so
    // raising rho* lifts the ceiling (rho* x capacity) AND lowers the per-demand multiplier
    // (demand x H, 3x -> 2x). The second effect is what freed capacity on lightly-loaded nodes. A
    // topology with a different load distribution could net out the other way.
    urllcHeadroom = 1.0 / urllcRhoStar;

    // One self-describing line per run. Without it a log identifies only its arm and link table,
    // so anything parsing results has to infer the rest from the output directory's name - and a
    // filename is not a schema. Keep it machine-greppable: `key=value`, space separated.
    cout << "config: router=" << router << " gaSeed=" << gaSeed << " rngRun=" << rngRun
         << " grid=" << gridWidth << " rhoStar=" << urllcRhoStar << " H=" << urllcHeadroom
         << " F=" << embbFloor << " mmtcH=" << mmtcHeadroom << " secondRound=" << secondRound
         << " fixedShare=" << (fixedShare.empty() ? string("-") : fixedShare)
         << " demand=" << embbDemand << "/" << urllcDemand << "/" << mmtcDemand
         << " linkTable=" << (linkTable.empty() ? "(warm-up)" : linkTable)
         // Which queue the run actually used. Without it an allocation run and the no-slicing
         // baseline are indistinguishable in the data, since every other field is identical.
         << " qdisc=" << (sliceQdisc ? "slice" : plainQdisc)
         << " sliceAqm=" << (sliceQdisc ? sliceAqm : string("-"))
         << " B=" << admitBudgetMax << "\n";

    QueueDiscContainer sliceQdiscs;
    SetSliceAqm(sliceAqm); // must precede the install; the tree is built there
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
    ctrlCfg.secondRound = secondRound;
    if (!fixedShare.empty())
    {
        stringstream fs(fixedShare);
        string tok;
        for (int s2 = 0; s2 < nsga::NUM_SLICES && getline(fs, tok, ','); ++s2)
        {
            ctrlCfg.fixedShare[s2] = stod(tok);
        }
    }
    array<uint64_t, nsga::NUM_SLICES> srcPkts = {};
    ctrlCfg.aiStep = aiStep;
    ctrlCfg.mdFactor = mdFactor;
    ctrlCfg.radioTargetPkts = radioTarget;
    ctrlCfg.period = ctrlPeriod;
    ctrlCfg.admitBudgetMax = admitBudgetMax;
    ctrlCfg.verbose = ctrlVerbose;

    InstallLossLedger(mesh); // name every frame the medium destroys; see loss-ledger.h
    FlowMonitorHelper flowmonHelper;
    // FlowMonitor declares a packet lost once it has gone MaxPerHopDelay unseen (default 10 s),
    // erases its tracking, and then IGNORES it if it does arrive (ReportLastRx returns early on
    // an untracked packet). mMTC's bucket alone can hold ~6 s at its cap, so the default turned
    // late deliveries into phantom loss AND dropped them out of the delay average. Raise it well
    // past any queueing delay the harness can produce; it is a measurement window, not a timeout.
    // 0 means derive it from the run's own span, so it can never fall behind --drainSec.
    const double lossWindowSec =
        (maxPerHopDelay > 0.0) ? maxPerHopDelay : 1.0 + trafficSec + drainSec;
    flowmonHelper.SetMonitorAttribute("MaxPerHopDelay", TimeValue(Seconds(lossWindowSec)));
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
        // Member counts scale with the grid so one configuration runs at every size from 10x10 to
        // 32x32. The grid is quartered: one quarter per slice, the fourth left with no members at
        // all so there are always pure relays to route THROUGH. Without that reserve, a bigger
        // grid would raise the member count and the offered load together and the sweep would
        // measure two changes at once.
        //
        // An odd quarter loses one node rather than rounding up: members are consumed as
        // source->destination pairs, so an odd count leaves one member with nobody to talk to. The
        // dropped nodes join the relay reserve, which is why it is not exactly a quarter -- at
        // 10x10 the quarter is 25, so each slice takes 24 and the reserve holds 28 of 100.
        if (Me == 0 && Mu == 0 && Mc == 0)
        {
            uint32_t perSlice = numNodes / 4;
            perSlice -= perSlice % 2;
            if (perSlice == 0)
            {
                cout << "error: grid too small to derive member counts (" << numNodes
                     << " nodes gives 0 per slice) - give --Me/--Mu/--Mc explicitly\n";
                return 1;
            }
            Me = Mu = Mc = perSlice;
            cout << "members derived from the grid: " << numNodes << " nodes / 4 = " << perSlice
                 << " per slice (even), " << (numNodes - 3 * perSlice) << " left as relay-only\n";
        }
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
        session.satRate = satRate;
        session.traceR = traceR;
        session.guardSec = guardSec;
        session.pktSize = pktSize;
        session.maxRounds = maxRounds;
        session.sampleEdges = sampleEdges;
        session.flushSender = flushSender;
        Simulator::Schedule(Seconds(warmupStart), &BeginWarmup, &session);
    };
    auto writeLinkTable = [&]() {
        vector<LinkMeasurement> table =
            ExtractLinkTable(session.ctx, session.classifier, flowMonitor);
        MedianCorrectOutliers(table);
        WriteLinkTable(table, resultsDir + "linktable.csv");
    };


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
        nsga::g_obj0Alloc = (obj0 == "alloc");
        Graph g = Graph::FromLinkTable(tablePath, numNodes);
        Params params{gaPop, gaGens, 0.1, t0Ms};
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
            cout << "pick: eMBB_spare=" << pick.obj[0] << " Mb/s  URLLC_delay=" << pick.obj[1]
                 << " ms  mMTC_links=" << static_cast<int>(pick.obj[2])
                 << "  stranded=" << pick.stranded << "  saturated=" << pick.saturated
                 << "\n";
            // Hop count per slice, and what the headroom formula makes of it. H = T*/(T* - D)
            // with D = N x T0 needs the WORST pair's hop count, not the mean: a headroom sized
            // for the average leaves half the pairs over budget by construction. Printed because
            // the formula is not what currently sets urllcHeadroom -- 1.64 is passed in by hand -
            // and this line is what says whether that number could have been derived at all.
            {
                array<Tree, nsga::NUM_SLICES> trees;
                for (int sl = 0; sl < nsga::NUM_SLICES; ++sl)
                {
                    trees[sl] = DecodeSlice(g, pick.seg[sl]);
                }
                LinkAirtime air = SpreadDemand(g, trees, slices);
                static const array<const char*, nsga::NUM_SLICES> nm = {"eMBB", "URLLC", "mMTC"};
                cout << "hops per pair (from the GA tree, before any traffic):\n";
                for (int sl = 0; sl < nsga::NUM_SLICES; ++sl)
                {
                    size_t mx = 0, sum = 0, n = 0;
                    for (const vector<EdgeKey>& path : air.pairPaths[sl])
                    {
                        mx = max(mx, path.size());
                        sum += path.size();
                        ++n;
                    }
                    cout << "  " << nm[sl] << ": pairs=" << n << " mean="
                         << (n ? 1.0 * sum / n : 0.0) << " max=" << mx;
                    if (sl == 1 && mx > 0)
                    {
                        double D = mx * t0Ms;
                        cout << "   D = max x T0 = " << mx << " x " << t0Ms << " = " << D
                             << " ms vs T* = " << urllcTargetMs << " ms -> H = ";
                        if (D < urllcTargetMs)
                        {
                            cout << (urllcTargetMs / (urllcTargetMs - D))
                                 << " (urllcHeadroom in use: " << urllcHeadroom << ")";
                        }
                        else
                        {
                            cout << "INFEASIBLE: the path spends the whole budget travelling, so "
                                    "no headroom meets T*";
                        }
                    }
                    cout << "\n";
                }
            }

            // Stranded members mean no feasible solution was found at all, and every objective
            // is then measured over whoever happened to stay connected.
            if (pick.stranded > 0)
            {
                cout << "WARNING: no fully connected solution found; " << pick.stranded
                     << " member(s) have no route. Objectives below are not comparable.\n";
            }
            cout << "installed " << routes.size() << " static host routes\n";

            // Phase 2: run the slice traffic on the injected routes (times are delay-from-now
            // since the sim clock already advanced through warm-up). Sources always offer their
            // full declared demand; policing happens at L2.5, never by lowering the offered load.
            if (sliceQdisc)
            {
                // Scale each band's AQM target by the flows multiplexed into it, so a slice queue
                // gives each flow what the baseline's per-flow FqCoDel queues give theirs. See
                // SetSliceLatencyTargets in traffic-control.h for the measured reason.
                size_t flowsPerSlice = 0;
                for (const nsga::Slice& sl : slices)
                {
                    flowsPerSlice = max(flowsPerSlice, sl.members.size() / 2);
                }
                const double target =
                    (cobaltTargetMs > 0.0)
                        ? cobaltTargetMs / 1e3
                        : CODEL_TARGET_SEC * static_cast<double>(max<size_t>(flowsPerSlice, 1));
                SetSliceLatencyTargets(sliceQdiscs, target);
                cout << "slice AQM target = " << target * 1e3 << " ms ("
                     << (cobaltTargetMs > 0.0 ? "given" : "5 ms x flows-per-slice") << ", "
                     << flowsPerSlice << " flows per slice)\n";

                // Capacity is MEASURED, per node, for the whole run -- never computed from a
                // model. `EffectiveCapacity` (link R / hops x a reuse factor) used to supply it
                // and was ~7x out (28.6 ms predicted against 168 measured); it was deleted on
                // 2026-10-11 along with the `--enforce` source-cap arm it served, rather than
                // repaired. Sources stay at full declared demand; the buckets do the policing.
                ctrl.Start(mesh.devices,
                           sliceQdiscs,
                           {slices[0].demandMbps, slices[1].demandMbps, slices[2].demandMbps},
                           ctrlCfg,
                           1.0 + ctrlCfg.period, // one period after the traffic starts
                           trafficSec);
                cout << "per-node capacity control ON (AIMD +" << ctrlCfg.aiStep << "/x"
                     << ctrlCfg.mdFactor << " on radio queue > " << ctrlCfg.radioTargetPkts
                     << " pkt, every " << ctrlCfg.period << "s; Headroom=" << urllcHeadroom
                     << ", F=" << embbFloor << ", mmtcHeadroom=" << mmtcHeadroom << ")\n";
            }
            for (int i = 0; i < nsga::NUM_SLICES; ++i)
            {
                InstallSliceTraffic(mesh, interfaces, slices[i], slices[i].demandMbps, bidir, 1.0,
                                    1.0 + trafficSec,
                                    pktSize);
            }
            // The drain is a condition, not a constant: poll until the queues are empty, and
            // treat drainSec as the safety cap. Stop() is still armed at the cap in case the
            // poller never fires.
            // Open the measured window measureSec before the sources stop, and gate it on the
            // queues having reached equilibrium first. Everything before this point is transient.
            vector<double> winLens;
            for (size_t p = 0; p < measureSec.size();)
            {
                size_t c = measureSec.find(',', p);
                double v = stod(measureSec.substr(p, c == string::npos ? string::npos : c - p));
                if (v > 0.0 && v <= trafficSec)
                {
                    winLens.push_back(v);
                }
                if (c == string::npos)
                {
                    break;
                }
                p = c + 1;
            }
            NS_ABORT_MSG_IF(winLens.empty(), "--measureSec gave no window inside --trafficSec");
            sort(winLens.begin(), winLens.end(), greater<double>()); // longest opens first
            vector<FlowWindow> windows(winLens.size());
            ScheduleSettleWatch(mesh, 1.0, 0.05, stallSec);
            for (size_t w = 0; w < winLens.size(); ++w)
            {
                // OccupancyTrend() stops the watcher, so only the LAST (shortest) window reads
                // it; the others print no note rather than repeating an unmeasured claim.
                Simulator::Schedule(Seconds(1.0 + trafficSec - winLens[w]),
                                    [&windows, &mesh, w, flowMonitor, stallSec, n = winLens.size()]() {
                                        OpenFlowWindow(mesh,
                                                       flowMonitor,
                                                       windows[w],
                                                       w + 1 == n ? OccupancyTrend(stallSec)
                                                                  : string());
                                    });
            }
            // Close every window the instant the sources stop, before the drain changes anything.
            Simulator::Schedule(Seconds(1.0 + trafficSec), [&windows, &mesh, settleGrowth]() {
                for (FlowWindow& w : windows)
                {
                    CloseFlowWindow(mesh, w, settleGrowth);
                }
            });
            ScheduleDrain(mesh, 1.0 + trafficSec, 0.05, drainSec, stallSec, drainReap);
            Simulator::Stop(Seconds(1.0 + trafficSec + drainSec));
            Simulator::Run();
            cout << "sim ended at t=" << Simulator::Now().GetSeconds() << "s\n";
            for (size_t w = 0; w < windows.size(); ++w)
            {
                cout << "\n--- measured over the last " << winLens[w] << " s of traffic ---\n";
                srcPkts = PrintSliceResults(
                    slices, flowMonitor,
                    DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier()), winLens[w],
                    &windows[w]);
            }
            if (sliceQdisc)
            {
                PrintSliceQdiscStats(sliceQdiscs, srcPkts);
                PrintCapVsOffered(sliceQdiscs, trafficSec);
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
                                warmupStart + trafficSec);
        }
        Simulator::Stop(Seconds(warmupStart + trafficSec + 2.0));
        Simulator::Run();
        PrintSliceResults(slices, flowMonitor,
                          DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier()),
                          trafficSec);
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
