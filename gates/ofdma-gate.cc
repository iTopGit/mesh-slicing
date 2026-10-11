// Gate: does the modeled OFDMA RU split actually give URLLC its own lane? The whole
// point of slicing is that URLLC's latency must NOT depend on how hard eMBB pushes.
// This sweeps eMBB load and shows the OLD shared-channel URLLC delay swinging (up to
// SAT) while the NEW per-RU URLLC delay stays flat — URLLC isolated on its own share.
#include "../ofdma/allocator.h"

#include "ns3/core-module.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace ns3;
using namespace std;
using namespace nsga;

// Shared-channel (no slicing) delay: every slice on the link adds airtime to one U.
static double
SharedDelay(double C, double R, double demandSum)
{
    double U = demandSum / R;
    return (U < 1.0) ? C / (1.0 - U) : INFINITY;
}

// A slice's delay on its OWN RU: only its own load fills its own share.
static double
RuDelay(double C, double demand, double share)
{
    if (share <= 0.0)
    {
        return INFINITY;
    }
    double U = demand / share;
    return (U < 1.0) ? C / (1.0 - U) : INFINITY;
}

static string
Fmt(double ms)
{
    if (isinf(ms))
    {
        return "SAT";
    }
    ostringstream o;
    o << fixed << setprecision(2) << ms;
    return o.str();
}

int
main(int argc, char* argv[])
{
    double R = 51.0, urllc = 10.0, mmtc = 4.0, C = 4.53, urllcHeadroom = 2.0;
    CommandLine cmd;
    cmd.AddValue("R", "Link capacity Mb/s", R);
    cmd.AddValue("urllc", "URLLC demand Mb/s", urllc);
    cmd.AddValue("mmtc", "mMTC demand Mb/s", mmtc);
    cmd.AddValue("C", "Calibrated delay constant ms", C);
    cmd.AddValue("urllcHeadroom", "URLLC share = demand x this (>=1)", urllcHeadroom);
    cmd.Parse(argc, argv);

    vector<double> embbLoads = {5.0, 30.0, 60.0, 90.0};

    cout << fixed << setprecision(2);
    cout << "\n=== OFDMA allocation gate ===\n";
    cout << "URLLC gets its own RU -> its delay should be independent of eMBB load.\n";
    cout << "R=" << R << " Mb/s  URLLC=" << urllc << "  mMTC=" << mmtc << "  C=" << C
         << " ms  urllcHeadroom=" << urllcHeadroom << "\n\n";

    cout << " eMBB load | OLD shared URLLC delay | NEW OFDMA URLLC delay | URLLC RU | eMBB RU\n";
    double newRef = -1.0;
    bool served = true, flat = true;
    for (double e : embbLoads)
    {
        array<double, NUM_SLICES> demand = {e, urllc, mmtc};
        array<double, NUM_SLICES> share = ofdma::Allocate(R, demand, urllcHeadroom);
        double oldD = SharedDelay(C, R, e + urllc + mmtc);
        double newD = RuDelay(C, urllc, share[1]);
        if (share[1] < urllc - 1e-9)
        {
            served = false;
        }
        if (newRef < 0)
        {
            newRef = newD;
        }
        else if (fabs(newD - newRef) > 1e-6)
        {
            flat = false;
        }
        cout << "   " << setw(5) << e << "   |   " << setw(8) << Fmt(oldD) << "             |   "
             << setw(8) << Fmt(newD) << "           |  " << setw(6) << share[1] << "  | " << setw(6)
             << share[0] << "\n";
    }

    // Representative contended split (eMBB=60): show every slice's share + throughput.
    cout << "\ncontended split (eMBB=60):\n";
    array<double, NUM_SLICES> d = {60.0, urllc, mmtc};
    array<double, NUM_SLICES> s = ofdma::Allocate(R, d, urllcHeadroom);
    const char* nm[NUM_SLICES] = {"eMBB ", "URLLC", "mMTC "};
    cout << "  slice   demand   RU share   throughput=min(demand,share)\n";
    for (int i = 0; i < NUM_SLICES; ++i)
    {
        cout << "  " << nm[i] << "    " << setw(5) << d[i] << "     " << setw(6) << s[i]
             << "        " << setw(6) << min(d[i], s[i]) << "\n";
    }

    cout << "\nURLLC RU >= demand in every case (served): " << (served ? "PASS" : "FAIL") << "\n";
    cout << "NEW URLLC delay independent of eMBB load (flat): " << (flat ? "PASS" : "FAIL") << "\n";
    return (served && flat) ? 0 : 1;
}
