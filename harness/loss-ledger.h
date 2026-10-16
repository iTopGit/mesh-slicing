#ifndef LOSS_LEDGER_H
#define LOSS_LEDGER_H

#include "topology.h"

#include <cstdint>
#include <string>

/**
 * Names every packet the medium destroys, so a loss figure stops being "somewhere below us".
 *
 * After the URLLC bucket was sized from its loss target the queue discs drop almost nothing, yet
 * ~19% of URLLC packets still fail end to end: ~3.4% of frame transmissions die per hop over 4.94
 * hops. Nothing in the harness said where, and the fitness has no loss term, so the loss could not
 * be attributed to routing, allocation or the radio.
 *
 * ns-3 already separates the causes: `WifiMac`'s `DroppedMpdu` trace carries a `WifiMacDropReason`
 * that is exactly the four candidates - queue full, lifetime expired, retry limit reached, stale
 * QoS packet - and `Ipv4L3Protocol`'s `Drop` trace carries its own reason, which catches a missing
 * route. Hook both on every node and count.
 *
 * Each answer sends the work somewhere different, which is the point: a retry-limit verdict means
 * the medium is genuinely colliding and the fix is routing or offered load; an expired-lifetime
 * verdict means frames waited too long, so the fix is back in the rates and depths; a failed
 * enqueue means bursts beat the radio's own 500-packet queue; a route drop is a routing bug.
 */
void InstallLossLedger(const MeshTopology& mesh);

/** Print the per-cause totals. Call after Simulator::Run. */
void PrintLossLedger();

/**
 * Packets the nodes are still holding when the run ends - queue discs plus each radio's own
 * AC_BE queue. A packet stuck here was counted as transmitted by FlowMonitor and never arrives,
 * yet no drop trace fires, so it reads as unexplained loss. Extra drain time does not release it:
 * WifiMacQueue only reaps an over-lifetime frame on the next enqueue, and once the sources stop
 * nothing enqueues again - the same lazy-reap behaviour that corrupted the warm-up link table.
 */
void PrintHeldPackets(const MeshTopology& mesh);

/** Total packets the nodes hold right now (queue discs + radio AC_BE queues). */
std::uint32_t HeldPackets(const MeshTopology& mesh,
                          std::uint32_t* inQdiscs = nullptr,
                          std::uint32_t* inRadios = nullptr);

/** Sweep every radio's expired MPDUs now. See the comment on the definition. */
void ReapExpired(const MeshTopology& mesh);

/**
 * Watch the held-packet count from `startSec` so the measurement window can tell whether the
 * queues had reached equilibrium before it opened. Queues at equilibrium means arrivals equal
 * departures, which means occupancy stops RISING -- the same observable the drain uses, so this
 * introduces no new constant.
 */
void ScheduleSettleWatch(const MeshTopology& mesh,
                         double startSec,
                         double pollSec = 0.05,
                         double stallSec = 2.0);

/**
 * Occupancy over the traffic so far, as a printable line: recent mean, whole-traffic mean, peak
 * and their ratio. Call once, as the window opens -- it stops the watcher. REPORTED, not judged:
 * see the comment block in loss-ledger.cc for the two boolean versions that failed and why a
 * yes/no needs a tolerance constant this project will not invent. Steady state is established by
 * agreement between nested measurement windows, not by this line.
 */
std::string OccupancyTrend(double stallSec);

/**
 * End the run when the queues are empty rather than after a fixed quiet time.
 *
 * RFC 2544 does ask for a drain - "wait for two seconds for any residual frames to be received" -
 * but two seconds is a figure for a switch whose buffer holds milliseconds, while a token bucket
 * here holds seconds: at 4x load a 2 s drain left 18.7% of all packets inside the queue discs, and
 * every delay and loss figure taken that way is truncated. So keep the requirement and drop the
 * constant. Polls from `startSec` every `pollSec`, stops as soon as nothing is held, and gives up
 * at `capSec` - which is a safety limit, not a drain time.
 *
 * `stallSec` is how long the held count may sit UNCHANGED before the queues are declared
 * not-draining; it is exposed so the verdict's independence from it can be demonstrated rather
 * than asserted. See the comment on `g_stallSec` in loss-ledger.cc.
 */
void ScheduleDrain(const MeshTopology& mesh,
                   double startSec,
                   double pollSec = 0.05,
                   double capSec = 300.0,
                   double stallSec = 2.0,
                   bool reap = true);

/**
 * Report the drain: how long the queues took to empty, or that the run is INVALID because the cap
 * was reached with packets still held. Returns true if the run is usable. The time is printed
 * because it is the "settling time used for data deletion" that Perrone, Yuan & Nicol (Winter
 * Simulation Conference 2003, section 5) require a study to state.
 */
bool PrintDrain(const MeshTopology& mesh, double capSec = 120.0);

#endif // LOSS_LEDGER_H
