#ifndef TRAFFIC_CONTROL_H
#define TRAFFIC_CONTROL_H

#include <cstdint>

/**
 * Each slice's packets are tagged with the IP ToS below so they can be identified
 * (per-flow reporting, and classification if a shaper is used). Slice differentiation
 * itself is NOT done at the MAC — EDCA does not differentiate slices in ns-3 (the
 * priority is stripped before the Wi-Fi MAC). Enforcement is at the control plane:
 * allocate each slice a capacity share summing below the link ceiling and rate-shape
 * its source to that share. The ToS values still map to distinct socket priorities
 * (Socket::IpTos2Priority), useful for a classful shaper.
 */
namespace SliceTos
{
constexpr uint8_t Urllc = 16; // low-latency slice
constexpr uint8_t Embb = 24;  // high-throughput slice
constexpr uint8_t Mmtc = 8;   // background slice
} // namespace SliceTos

#endif // TRAFFIC_CONTROL_H
