#ifndef RUN_MODE_H
#define RUN_MODE_H

#include <string>

// Validate + normalize the harness run-mode flags: reject two routing methods at once, apply the
// implied deps (gaRoute needs the warm-up table; olsr drives a slice test), and require at least
// one mode. Mutates warmup/sliceTest in place; returns false on an illegal / empty combo (main
// should exit 1). Why the modes are shaped this way: PLAN.md "Harness run modes".
bool ResolveRunMode(bool olsr, bool gaRoute, bool& warmup, bool& sliceTest);

// Isolate this run's output so parallel sims don't clobber each other's pcap/flowmon/linktable:
// give every concurrent run its own --resultsDir. Appends the trailing slash the callers
// concatenate onto and creates the dir. Mutates resultsDir in place.
void PrepareResultsDir(std::string& resultsDir);

#endif // RUN_MODE_H
