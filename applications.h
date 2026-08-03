#ifndef APPLICATIONS_H
#define APPLICATIONS_H

#include "topology.h"

#include "ns3/ipv4-interface-container.h"

#include <cstdint>
#include <string>

/**
 * Installs a UdpEchoServer on mesh.nodes[dst] and a UdpEchoClient on
 * mesh.nodes[src] that sends it echo requests. Uses `interfaces` (from
 * InstallInternetStack) to look up dst's address.
 */
void InstallEchoApps(const MeshTopology& mesh,
                      const ns3::Ipv4InterfaceContainer& interfaces,
                      uint32_t src,
                      uint32_t dst);

/**
 * One network slice under test: a UDP flow between two access nodes, marked with an
 * IP ToS so it can be identified per-flow (and classified if a shaper is used).
 * `src`/`dst` are node indices; slice members are fixed here for testing (random
 * selection comes later, per PLAN.md).
 */
struct Slice
{
    std::string name;      // eMBB / URLLC / mMTC
    uint32_t src;          // source access node
    uint32_t dst;          // dest access node
    std::string dataRate;  // offered UDP rate, e.g. "5Mb/s"
    uint8_t tos;           // IP ToS -> socket priority (slice id / classification)
    uint16_t port;
};

/** Install one slice's UDP source (700-byte payload, ToS-marked) and its sink. */
void InstallSliceTraffic(const MeshTopology& mesh,
                         const ns3::Ipv4InterfaceContainer& interfaces,
                         const Slice& slice,
                         double startSec,
                         double stopSec);

#endif // APPLICATIONS_H
