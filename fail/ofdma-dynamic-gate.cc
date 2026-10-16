// SUPERSEDED for harness use by gates/capacity-feedback-gate.cc. Kept: the result below is
// real and the loop mechanics are proven. But this controller CANNOT be wired into the harness
// as it stands, for three reasons:
//   1. it retunes OnOff::DataRate -- the SOURCE cap -- while the harness now enforces at L2.5;
//      the equivalent call there is SetSliceRates(qdiscs, ...);
//   2. it senses DEMAND, and harness demand is constant, so the loop would recompute the same
//      answer forever;
//   3. it is HANDED R as a constant, whereas the harness derives R from EffectiveCapacity, and
//      that is precisely the quantity that is wrong (12.19 modelled vs ~8 delivered on 5x5).
// capacity-feedback-gate.cc keeps the loop and changes what it senses: capacity, not demand.
//
// Gate: dynamic OFDMA recompute. Demand changes mid-run (URLLC steps up); a periodic
// controller re-runs ofdma::Allocate on the CURRENT demand every T seconds and retunes
// each slice's rate-shaped source live (SetAttribute("DataRate") — ns-3 reads m_cbrRate
// per packet, so the change takes hold within one packet). Compares STATIC (shares frozen
// at t0) vs DYNAMIC (shares tracked): when URLLC demand grows, static leaves URLLC at 100%
// of its stale RU -> the shared channel saturates -> URLLC delay blows up; dynamic restores
// URLLC headroom (squeezing eMBB off the residual) -> channel stays unsaturated -> URLLC
// protected. Burst-window delay is measured by snapshotting FlowMonitor at the step.
#include "../ofdma/allocator.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

using namespace ns3;
using namespace std;
using namespace nsga;

static const uint16_t PORT[NUM_SLICES] = {5001, 5002, 5003};
static const char* NM[NUM_SLICES] = {"eMBB ", "URLLC", "mMTC "};

struct SliceResult
{
    double thpt = 0.0;  // burst-window Mb/s
    double delay = 0.0; // burst-window ms
};

// Install one slice's rate-shaped UDP source, capped to initMbps. Returns the OnOff app so
// the controller can retune its rate mid-run.
static Ptr<Application>
InstallShapedFlow(Ptr<Node> src, Ptr<Node> dst, Ipv4Address dstAddr, uint16_t port,
                  double initMbps, uint32_t pktSize, double start, double stop)
{
    ostringstream rate;
    rate << static_cast<long>(initMbps * 1e6) << "bps";
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    onoff.SetConstantRate(DataRate(rate.str()), pktSize);
    ApplicationContainer s = onoff.Install(src);
    s.Start(Seconds(start));
    s.Stop(Seconds(stop));

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer r = sink.Install(dst);
    r.Start(Seconds(start - 0.5));
    r.Stop(Seconds(stop + 0.5));
    return s.Get(0);
}

// The dynamic controller: holds each slice's source, the live demand, and re-splits + retunes
// on a timer. STATIC mode keeps the frozen t0 shares; DYNAMIC re-runs the allocator each tick.
struct Controller
{
    array<Ptr<Application>, NUM_SLICES> app;
    array<double, NUM_SLICES> demand;
    array<double, NUM_SLICES> frozen;
    bool dynamic = false;
    double R = 51.0, urllcHeadroom = 2.0, T = 0.5, tEnd = 7.0;

    Ptr<FlowMonitor> fm;
    Ptr<Ipv4FlowClassifier> cl;
    array<double, NUM_SLICES> snapDelay = {0, 0, 0}; // delaySum(s) at burst start
    array<uint64_t, NUM_SLICES> snapRx = {0, 0, 0};  // rxPackets at burst start

    array<double, NUM_SLICES> Shares() const
    {
        return dynamic ? ofdma::Allocate(R, demand, urllcHeadroom) : frozen;
    }

    void Retune()
    {
        array<double, NUM_SLICES> share = Shares();
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            double cap = min(demand[s], share[s]);
            ostringstream r;
            r << static_cast<long>(cap * 1e6) << "bps";
            app[s]->SetAttribute("DataRate", DataRateValue(DataRate(r.str())));
        }
    }

    void Tick()
    {
        Retune();
        if (Simulator::Now().GetSeconds() + T <= tEnd + 1e-9)
        {
            Simulator::Schedule(Seconds(T), &Controller::Tick, this);
        }
    }

    void BumpDemand(int slice, double v)
    {
        demand[slice] = v;
        Retune(); // apply immediately, don't wait for the next tick
    }

    // Record cumulative delaySum / rxPackets per slice at the burst step, so the burst-window
    // average can be recovered as (final - snapshot).
    void Snapshot()
    {
        for (const auto& [id, st] : fm->GetFlowStats())
        {
            uint16_t p = cl->FindFlow(id).destinationPort;
            for (int s = 0; s < NUM_SLICES; ++s)
            {
                if (p == PORT[s])
                {
                    snapDelay[s] = st.delaySum.GetSeconds();
                    snapRx[s] = st.rxPackets;
                }
            }
        }
    }
};

// One scenario: 3 slices share node0->node1 with a demand step; static or dynamic control.
// Returns per-slice burst-window {thpt, delay}.
static array<SliceResult, NUM_SLICES>
RunScenario(bool dynamic, const array<double, NUM_SLICES>& demandLow, int burstSlice,
            double burstVal, double R, double urllcHeadroom, double C, uint32_t pktSize)
{
    (void)C;
    const double tStart = 1.0, tBurst = 2.0, tStop = 7.0, tPer = 0.5;

    NodeContainer nodes;
    nodes.Create(2);

    MobilityHelper mob;
    mob.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY",
                             DoubleValue(0.0), "DeltaX", DoubleValue(50.0), "DeltaY",
                             DoubleValue(0.0), "GridWidth", UintegerValue(2), "LayoutType",
                             StringValue("RowFirst"));
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(nodes);

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("ChannelSettings", StringValue("{0, 80, BAND_5GHZ, 0}"));

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
    Ipv4Address d = iface.GetAddress(1);

    Controller* ctrl = new Controller();
    ctrl->demand = demandLow;
    ctrl->frozen = ofdma::Allocate(R, demandLow, urllcHeadroom); // t0 split (used verbatim in STATIC)
    ctrl->dynamic = dynamic;
    ctrl->R = R;
    ctrl->urllcHeadroom = urllcHeadroom;
    ctrl->T = tPer;
    ctrl->tEnd = tStop;

    array<double, NUM_SLICES> share0 = ctrl->frozen;
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        double cap0 = min(demandLow[s], share0[s]);
        ctrl->app[s] =
            InstallShapedFlow(nodes.Get(0), nodes.Get(1), d, PORT[s], cap0, pktSize, tStart, tStop);
    }

    FlowMonitorHelper fmh;
    ctrl->fm = fmh.InstallAll();
    ctrl->cl = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());

    Simulator::Schedule(Seconds(tStart), &Controller::Tick, ctrl);
    Simulator::Schedule(Seconds(tBurst), &Controller::Snapshot, ctrl);
    Simulator::Schedule(Seconds(tBurst), &Controller::BumpDemand, ctrl, burstSlice, burstVal);

    Simulator::Stop(Seconds(tStop + 1.0));
    Simulator::Run();

    array<SliceResult, NUM_SLICES> out;
    for (const auto& [id, st] : ctrl->fm->GetFlowStats())
    {
        uint16_t p = ctrl->cl->FindFlow(id).destinationPort;
        for (int s = 0; s < NUM_SLICES; ++s)
        {
            if (p != PORT[s])
            {
                continue;
            }
            uint64_t rx = st.rxPackets - ctrl->snapRx[s];
            double dsum = st.delaySum.GetSeconds() - ctrl->snapDelay[s];
            double window = tStop - tBurst;
            out[s].delay = (rx > 0) ? dsum / rx * 1e3 : 0.0;
            out[s].thpt = rx * pktSize * 8.0 / window / 1e6;
        }
    }
    Simulator::Destroy();
    delete ctrl;
    return out;
}

static void
PrintTable(const char* label, const array<SliceResult, NUM_SLICES>& r)
{
    cout << "\n" << label << "  (burst window)\n";
    cout << "  slice   thpt(Mb/s)   delay(ms)\n";
    for (int s = 0; s < NUM_SLICES; ++s)
    {
        cout << "  " << NM[s] << "   " << setw(8) << r[s].thpt << "     " << setw(8) << r[s].delay
             << "\n";
    }
}

int
main(int argc, char* argv[])
{
    double embb = 40.0, urllcLow = 10.0, urllcHigh = 20.0, mmtc = 4.0;
    double R = 51.0, urllcHeadroom = 2.0, C = 4.53;
    uint32_t pktSize = 700;
    CommandLine cmd;
    cmd.AddValue("embb", "eMBB demand Mb/s (steady, elastic)", embb);
    cmd.AddValue("urllcLow", "URLLC demand before the step Mb/s", urllcLow);
    cmd.AddValue("urllcHigh", "URLLC demand after the step Mb/s", urllcHigh);
    cmd.AddValue("mmtc", "mMTC demand Mb/s", mmtc);
    cmd.AddValue("R", "Link capacity Mb/s", R);
    cmd.AddValue("urllcHeadroom", "URLLC share = demand x this (>=1)", urllcHeadroom);
    cmd.AddValue("C", "Model delay constant ms", C);
    cmd.AddValue("pktSize", "UDP payload bytes", pktSize);
    cmd.Parse(argc, argv);

    array<double, NUM_SLICES> demandLow = {embb, urllcLow, mmtc};
    const int URLLC = 1;

    cout << fixed << setprecision(3);
    cout << "=== OFDMA dynamic-recompute gate (3 slices, one ax link) ===\n";
    cout << "demand step: URLLC " << urllcLow << " -> " << urllcHigh << " Mb/s at t=2s   (eMBB "
         << embb << " steady, mMTC " << mmtc << ", R=" << R << ", urllcHeadroom=" << urllcHeadroom << ")\n";

    array<double, NUM_SLICES> sLow = ofdma::Allocate(R, demandLow, urllcHeadroom);
    array<double, NUM_SLICES> dHigh = {embb, urllcHigh, mmtc};
    array<double, NUM_SLICES> sHigh = ofdma::Allocate(R, dHigh, urllcHeadroom);
    cout << "\ncontrol decisions (allocator):\n";
    cout << "  before step  RU: eMBB=" << sLow[0] << " URLLC=" << sLow[1] << " mMTC=" << sLow[2]
         << "\n";
    cout << "  after step   RU: eMBB=" << sHigh[0] << " URLLC=" << sHigh[1] << " mMTC=" << sHigh[2]
         << "   <- DYNAMIC moves here; STATIC stays on the 'before' row\n";

    array<SliceResult, NUM_SLICES> stat =
        RunScenario(false, demandLow, URLLC, urllcHigh, R, urllcHeadroom, C, pktSize);
    PrintTable("STATIC  (shares frozen at t0):", stat);
    array<SliceResult, NUM_SLICES> dyn =
        RunScenario(true, demandLow, URLLC, urllcHigh, R, urllcHeadroom, C, pktSize);
    PrintTable("DYNAMIC (recompute every 0.5s on live demand):", dyn);

    cout << "\nURLLC burst delay:  STATIC=" << stat[URLLC].delay << " ms  ->  DYNAMIC="
         << dyn[URLLC].delay << " ms\n";

    bool protectedOk = dyn[URLLC].delay < stat[URLLC].delay;
    bool servedOk = dyn[URLLC].thpt >= urllcHigh * 0.9;
    cout << "\ndynamic protects URLLC through the demand step: " << (protectedOk ? "PASS" : "FAIL")
         << " (STATIC " << stat[URLLC].delay << " -> DYNAMIC " << dyn[URLLC].delay << " ms)\n";
    cout << "URLLC still served after the step (thpt >= 90% demand): " << (servedOk ? "PASS" : "FAIL")
         << " (" << dyn[URLLC].thpt << " Mb/s)\n";
    return (protectedOk && servedOk) ? 0 : 1;
}
