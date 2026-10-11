#include "run-mode.h"

#include <filesystem>
#include <iostream>

using namespace std;

// Error order relies on olsr/gaRoute being mutually exclusive: warmup is only forced true here
// when gaRoute is set (so olsr is already false), keeping the olsr&&warmup check safe.
bool
ResolveRunMode(bool olsr, bool gaRoute, bool& warmup, bool& sliceTest)
{
    if (olsr && gaRoute)
    {
        cout << "error: --olsr and --gaRoute are different routing methods; run one at a time\n";
        return false;
    }
    if (olsr && warmup)
    {
        cout << "error: --olsr is the OLSR test case; warm-up is always static (don't combine)\n";
        return false;
    }
    if (gaRoute)
    {
        warmup = true;
    }
    if (olsr)
    {
        sliceTest = true;
    }
    if (!warmup && !sliceTest)
    {
        cout << "error: pick a mode: --warmup, --sliceTest, --olsr, or --gaRoute\n";
        return false;
    }
    return true;
}

void
PrepareResultsDir(string& resultsDir)
{
    if (!resultsDir.empty() && resultsDir.back() != '/')
    {
        resultsDir += '/';
    }
    filesystem::create_directories(resultsDir);
}
