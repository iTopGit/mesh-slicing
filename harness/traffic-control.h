#ifndef TRAFFIC_CONTROL_H
#define TRAFFIC_CONTROL_H

#include "../nsga/individual.h" // nsga::NUM_SLICES + slice-index order

#include "ns3/net-device-container.h"
#include "ns3/queue-disc-container.h"

#include <array>
#include <cstdint>

/**
 * Per-slice DiffServ marks. Stamped on every packet at the source (applications.cc) and read
 * again at EVERY hop: a relay rebuilds the mark from the IP header, so a node four hops from
 * the sender still knows which slice a packet belongs to without any extra state. Nothing
 * rides along with the packet -- the header byte IS the carrier.
 *
 * The ToS byte does two unrelated jobs, and they are split here by bit range:
 *
 *     7 6 5 | 4 3 2 | 1 0
 *     \_UP_/  \slice/  \ECN/
 *
 *   bits 5-7  User Priority. ns-3 reads exactly these (`dscp >> 5` in SelectQueueByDSField,
 *             qos-utils.cc) and maps them to an EDCA access category. RFC 8325 defines the
 *             same three bits, so this half is standard, not an ns-3 habit.
 *   bits 2-4  slice ID. Free: the sender writes, everyone in between only reads.
 *   bits 0-1  ECN. NOT ours -- these exist so a congested node can write on a packet in
 *             flight, so a slice ID stored here could be silently overwritten mid-path. This
 *             is where we diverge from Marrone et al. (MeditCom 2025), who take all five low
 *             bits; their single-hop AP topology has no relay that could remark them, ours is
 *             all relays. Three bits give eight slice IDs against the three we need.
 *
 * Splitting the two jobs is the point. While the slice lived in bits 5-7 the slice number
 * also picked the access category, so eMBB and mMTC landed on AC_BK -- whose A-MPDU
 * aggregation ns-3 ships DISABLED (WifiMac BK_MaxAmpduSize defaults to 0 against 65535 for
 * AC_BE). Nobody chose that; it fell out of the numbering. Now every slice is marked UP 3 =
 * AC_BE, so all three get identical MAC treatment and aggregation, and the queue disc below
 * is the ONLY thing that tells them apart.
 *
 * Keep every slice on AC_BE or AC_BK. AC_VI and AC_VO cap their contention window at 15 and 7
 * (WifiMac::ConfigureDcf) instead of 1023, which a multi-hop chain cannot tolerate -- a relay
 * and its source never de-synchronise and the flow collapses.
 *
 * Slice ID 0 is deliberately unused, so an unmarked packet classifies to the uncapped control
 * band rather than silently joining eMBB.
 */
namespace SliceTos
{
constexpr std::uint8_t Embb = 100;  // 011 001 00 : UP 3, slice 1 (DSCP 25)
constexpr std::uint8_t Urllc = 104; // 011 010 00 : UP 3, slice 2 (DSCP 26 = AF31)
constexpr std::uint8_t Mmtc = 108;  // 011 011 00 : UP 3, slice 3 (DSCP 27)
} // namespace SliceTos

/** Slice ID carried in ToS bits 2-4. 0 means unmarked. */
constexpr int SliceIdFromTos(std::uint8_t tos)
{
    return (tos >> 2) & 0x7;
}

/**
 * PrioQueueDisc band per slice, indexed the allocator's way (0 = eMBB, 1 = URLLC, 2 = mMTC).
 * Band 0 is not a slice: it is the uncapped lane for ARP and anything else unmarked, which
 * must never be rate-limited or next-hop resolution stalls the moment a slice saturates.
 */
constexpr std::uint16_t SLICE_BAND_CTRL = 0;
inline constexpr std::array<std::uint16_t, nsga::NUM_SLICES> SLICE_BAND = {2, 1, 3};

/**
 * Band per SLICE ID (the bits-2-4 value), which is the allocator index plus one. Every ID
 * without a slice maps to the control band, so a missing or malformed mark is never policed
 * as somebody else's traffic. SLICE_ID_BAND[i + 1] == SLICE_BAND[i] by construction.
 */
inline constexpr std::array<std::uint16_t, 8> SLICE_ID_BAND = {0, 2, 1, 3, 0, 0, 0, 0};

/**
 * Install the per-slice queue disc on every radio: a PrioQueueDisc root that classifies on
 * the SliceTos mark, over one uncapped Fifo (control) and one TbfQueueDisc per slice.
 *
 * MUST be called after the internet stack is installed but BEFORE addresses are assigned:
 * Ipv4AddressHelper::Assign drops in the default queue disc when the slot is still empty,
 * and TrafficControlLayer wires each disc's SendCallback once, during initialization.
 *
 * The Tbf rates are left at a placeholder here — the shares are not known until the
 * warm-up has measured the links. Call SetSliceRates once they are.
 */
ns3::QueueDiscContainer InstallSliceQdiscs(const ns3::NetDeviceContainer& devices);

/**
 * Point each slice's token bucket at its allocated share (Mb/s), on every node. Safe to
 * call mid-simulation, which is the normal case: the harness only learns the shares after
 * the warm-up sweep, and the dynamic controller re-calls this to retune.
 *
 * Isolation holds only while the shares SUM TO AT MOST the real link capacity. A token
 * bucket is a ceiling, not a floor; the floor is manufactured by that sum. Oversubscribe
 * and the strict-priority root starves the lowest band, which is plain QoS again.
 */
/**
 * Set every slice band's AQM sojourn target, on every node.
 *
 * Why this is not left at Cobalt's 5 ms default. Both arms of the comparison use the SAME target,
 * but they apply it to different things: ns-3's FqCoDel hashes into 1024 flow queues, so the
 * no-slicing baseline measures sojourn PER FLOW, while a slice band is ONE queue carrying all of
 * that slice's flows. Twelve multiplexed flows build roughly twelve times the backlog, so the
 * aggregate queue crosses 5 ms far sooner and Cobalt drops traffic the baseline never touches --
 * measured at 9 Mb/s per slice as 26-37% loss across all three slices against the baseline's
 * 0.00%, including eMBB, whose rate limit (44.78 Mb/s against 1.99 offered) cannot possibly bind.
 *
 * So the target is scaled by the number of flows multiplexed into the band: `5 ms x flows`, where
 * 5 ms is Nichols & Jacobson's published CoDel value and the flow count is COUNTED from
 * `Slice::Pairs`. That keeps per-flow treatment equivalent to the baseline's and follows the grid
 * when member counts change, so it is a derivation rather than a tuned constant.
 */
/**
 * CoDel's published sojourn target, Nichols & Jacobson -- also ns-3's own default for CoDel,
 * FqCoDel and Cobalt. CITED, not chosen. It is the per-FLOW figure, which is why a slice band
 * carrying several flows scales it; see SetSliceLatencyTargets.
 */
constexpr double CODEL_TARGET_SEC = 0.005;

/**
 * Pick the Tbf's child discipline before InstallSliceQdiscs runs. Only peek-safe discs are
 * legal here -- Fifo, Cobalt, Red, PfifoFast, Prio, Mq -- because TbfQueueDisc reads the head
 * packet's size through Peek(); FqCoDel and CoDel peek by DEQUEUING and spin at 100% CPU.
 * See the comment block in InstallSliceQdiscs.
 */
void SetSliceAqm(const std::string& typeId);

void SetSliceLatencyTargets(const ns3::QueueDiscContainer& qdiscs, double targetSec);

void SetSliceRates(const ns3::QueueDiscContainer& qdiscs,
                   const std::array<double, nsga::NUM_SLICES>& limitMbps);

/**
 * Same, for ONE radio. The per-node controller needs this: capacity is a property of the
 * medium around a node, not of the field, so each node is allocated from its own reading.
 */
void SetSliceRatesOn(const ns3::QueueDiscContainer& qdiscs,
                     std::uint32_t index,
                     const std::array<double, nsga::NUM_SLICES>& limitMbps);

/**
 * Bytes offered to each slice's bucket on one radio, counted BEFORE any policing. This is
 * the node's local demand signal. It must be the received counter, never the dequeued one:
 * dequeued bytes are what the cap already allowed through, so feeding that back makes the
 * cap chase its own tail to zero (the same failure as capacity-feedback-gate's sensor (a)).
 */
std::array<std::uint64_t, nsga::NUM_SLICES> SliceOfferedBytes(
    const ns3::QueueDiscContainer& qdiscs,
    std::uint32_t index);

/**
 * Bytes each slice band is holding RIGHT NOW, as an instantaneous level rather than a counter.
 * The controller takes the larger of this and the tick's arrivals, because a limit sized from
 * arrivals alone can be below what the bucket already holds, and then the backlog can never
 * clear: measured at load 27, one URLLC node was offered 28.78x its own limit.
 */
std::array<std::uint64_t, nsga::NUM_SLICES> SliceBacklogBytes(
    const ns3::QueueDiscContainer& qdiscs,
    std::uint32_t index);

/**
 * Network-wide per-band totals (packets enqueued / dropped). A slice's drops here are its
 * own excess dying in its own bucket, which is the enforcement working; drops in the
 * control band mean the shares were set so low that ARP is starving.
 */
void PrintSliceQdiscStats(const ns3::QueueDiscContainer& qdiscs,
                          const std::array<std::uint64_t, nsga::NUM_SLICES>& srcPackets = {});

/**
 * Per slice, the cap against what each node carrying it was offered, then the same per node for
 * URLLC. A network-wide "policed = 0%" cannot tell a generous cap from one that is never reached;
 * the offered/cap ratio can, which is what says whether the allocator binds at this load. The
 * URLLC rows additionally separate the two causes of its loss: offered above cap needs a bigger
 * rate, drops while under cap need a deeper bucket.
 */
void PrintCapVsOffered(const ns3::QueueDiscContainer& qdiscs, double spanSec);

/**
 * Set the minimum cap a bucket may be given, derived as one MTU per control period: below that a
 * bucket cannot pass a full-size packet before the controller retunes it, so its backlog is
 * effectively frozen. Call it when the controller's period is known; the default derives from
 * 0.5 s. See the comment on `g_minLimitMbps` in traffic-control.cc.
 */
void SetMinLimitFromPeriod(double periodSec);

#endif // TRAFFIC_CONTROL_H
