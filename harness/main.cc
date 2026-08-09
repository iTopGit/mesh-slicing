#include "applications.h"
#include "internet.h"
#include "topology.h"
#include "traffic-control.h"
#include "warmup.h"

#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"

#include <filesystem>

using namespace ns3;
using namespace std;

int
main(int argc, char* argv[])
{
    uint32_t gridWidth = 10;  // columns; 10x10 = 100 nodes, per proposal
    uint32_t gridHeight = 0;  // rows; 0 = square (use gridWidth)
    double spacing = 50.0;    // meters
    uint32_t chWidth = 80;    // channel width (MHz)
    string resultsDir = "scratch/mesh-slicing/results/";
    bool verbose = false;     // log UdpEcho send/receive to stdout
    bool warmup = false;      // run the staggered per-link measurement sweep
    bool sliceTest = false;   // run the per-slice traffic test
    double warmupStart = 2.0; // sim time to begin the sweep / slice traffic
    double windowSec = 0.5;   // duration of each L and R measurement window
    double primeSec = 0.1;    // per-edge pre-roll (ARP/queue settle); 0 = skip
    int maxRounds = 3;        // warm-up retry rounds
    string embbRate = "4Mb/s"; // per-slice offered rates (raise to congest)
    string urllcRate = "1Mb/s";
    string mmtcRate = "2Mb/s";

    CommandLine cmd;
    cmd.AddValue("resultsDir", "Output dir for pcap/flowmon/linktable (give each parallel run its own)", resultsDir);
    cmd.AddValue("gridWidth", "Grid width (columns)", gridWidth);
    cmd.AddValue("gridHeight", "Grid height (rows); 0 = square (= gridWidth)", gridHeight);
    cmd.AddValue("spacing", "Distance between adjacent nodes (m)", spacing);
    cmd.AddValue("chWidth", "Channel width MHz (e.g. 20, 40, 80)", chWidth);
    cmd.AddValue("verbose", "Log UdpEcho send/receive to stdout", verbose);
    cmd.AddValue("warmup", "Run the staggered per-link warm-up sweep", warmup);
    cmd.AddValue("sliceTest", "Run the per-slice traffic test", sliceTest);
    cmd.AddValue("warmupStart", "Sim time (s) to begin the warm-up sweep / slice traffic", warmupStart);
    cmd.AddValue("windowSec", "Duration (s) of each warm-up measurement window", windowSec);
    cmd.AddValue("primeSec", "Unmeasured pre-roll (s) per edge (ARP/queue settle); 0 = skip", primeSec);
    cmd.AddValue("maxRounds", "Warm-up retry rounds", maxRounds);
    cmd.AddValue("embbRate", "eMBB slice offered rate", embbRate);
    cmd.AddValue("urllcRate", "URLLC slice offered rate", urllcRate);
    cmd.AddValue("mmtcRate", "mMTC slice offered rate", mmtcRate);
    cmd.Parse(argc, argv);

    // Parallel runs each need their own dir; create it (ofstream/pcap won't) and
    // keep the trailing slash the "dir + filename" concatenations rely on.
    if (!resultsDir.empty() && resultsDir.back() != '/')
    {
        resultsDir += '/';
    }
    std::filesystem::create_directories(resultsDir);

    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

    uint32_t rows = (gridHeight == 0) ? gridWidth : gridHeight;
    MeshTopology mesh = BuildGridAdhocAxTopology(gridWidth, rows, spacing, chWidth, resultsDir + "mesh");
    Ipv4InterfaceContainer interfaces = InstallInternetStack(mesh);

    if (!warmup && !sliceTest)
    {
        // Adjacent neighbors: node 0 (top-left) -> node 1. Single hop, simplest
        // sanity check that the network forwards at all.
        InstallEchoApps(mesh, interfaces, 0, 1);
    }

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> flowMonitor = flowmonHelper.InstallAll();

    // One slice per class under test (fixed members for now): all three share the
    // 0->2 path so they contend. Random membership + NSGA-II routing come later.
    vector<Slice> slices;
    if (sliceTest)
    {
        slices.push_back({"eMBB", 0, 2, embbRate, SliceTos::Embb, 5001});
        slices.push_back({"URLLC", 0, 2, urllcRate, SliceTos::Urllc, 5002});
        slices.push_back({"mMTC", 0, 2, mmtcRate, SliceTos::Mmtc, 5003});
        for (const Slice& s : slices)
        {
            InstallSliceTraffic(mesh, interfaces, s, warmupStart, warmupStart + 8.0);
        }
    }

    WarmupSession session;
    if (warmup)
    {
        // BeginWarmup enumerates edges geometrically, sweeps all of them serially,
        // then drives the retry rounds and sets its own stop time.
        session.mesh = mesh;
        session.interfaces = interfaces;
        session.classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
        session.monitor = flowMonitor;
        session.windowSec = windowSec;
        session.primeSec = primeSec;
        session.maxRounds = maxRounds;
        Simulator::Schedule(Seconds(warmupStart), &BeginWarmup, &session);
    }
    else if (sliceTest)
    {
        Simulator::Stop(Seconds(warmupStart + 10.0));
    }
    else
    {
        Simulator::Stop(Seconds(5.0)); // echo sanity path
    }

    Simulator::Run();

    flowMonitor->SerializeToXmlFile(resultsDir + "flowmon.xml", true, true);

    if (warmup)
    {
        vector<LinkMeasurement> table =
            ExtractLinkTable(session.ctx, session.classifier, flowMonitor);
        MedianCorrectOutliers(table);
        WriteLinkTable(table, resultsDir + "linktable.csv");
    }

    if (sliceTest)
    {
        // Map each flow back to its slice by dest port and print what it got.
        Ptr<Ipv4FlowClassifier> classifier =
            DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
        for (const auto& [id, st] : flowMonitor->GetFlowStats())
        {
            Ipv4FlowClassifier::FiveTuple ft = classifier->FindFlow(id);
            for (const Slice& s : slices)
            {
                if (ft.destinationPort != s.port)
                {
                    continue;
                }
                double dur = (st.timeLastRxPacket - st.timeFirstRxPacket).GetSeconds();
                double thpt = (dur > 0) ? (st.rxBytes * 8.0 / dur / 1e6) : 0.0;
                double delay = (st.rxPackets > 0) ? (st.delaySum.GetSeconds() / st.rxPackets * 1e3) : 0.0;
                double loss = (st.txPackets > 0)
                                  ? (100.0 * (st.txPackets - st.rxPackets) / st.txPackets)
                                  : 0.0;
                cout << "slice " << s.name << " " << s.src << "->" << s.dst
                     << ": tx=" << st.txPackets << " rx=" << st.rxPackets << " thpt=" << thpt
                     << "Mb/s delay=" << delay << "ms loss=" << loss << "%" << endl;
            }
        }
    }

    Simulator::Destroy();
    return 0;
}
