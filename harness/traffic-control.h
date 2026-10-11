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
void SetSliceRates(const ns3::QueueDiscContainer& qdiscs,
                   const std::array<double, nsga::NUM_SLICES>& shareMbps);

/**
 * Same, for ONE radio. The per-node controller needs this: capacity is a property of the
 * medium around a node, not of the field, so each node is allocated from its own reading.
 */
void SetSliceRatesOn(const ns3::QueueDiscContainer& qdiscs,
                     std::uint32_t index,
                     const std::array<double, nsga::NUM_SLICES>& shareMbps);

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

/** URLLC bucket depth, in packets. Default 24: the minimum that meets the cited PLR <= 1e-5 at
 *  the headroom's own rho* = 0.61 over ~3.4 hops. Must be called BEFORE InstallSliceQdiscs. */
void SetUrllcQueuePackets(std::uint32_t packets);

#endif // TRAFFIC_CONTROL_H
