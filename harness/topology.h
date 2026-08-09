#ifndef TOPOLOGY_H
#define TOPOLOGY_H

#include "ns3/net-device-container.h"
#include "ns3/node-container.h"

#include <string>

/**
 * A built mesh: the nodes (for stack/app installation, one Node per router) and
 * their Wi-Fi devices (for address/queue installation, one device per node, same
 * index order as `nodes`). The mesh is realized as an 802.11ax ad-hoc network with
 * L3 routing — a multi-hop wireless mesh at the network layer.
 */
struct MeshTopology
{
    ns3::NodeContainer nodes;
    ns3::NetDeviceContainer devices;
};

/**
 * Builds a gridWidth x gridHeight grid of 802.11ax nodes in ad-hoc mode at
 * `chWidth` MHz, spaced `spacing` meters apart on one shared 5 GHz channel
 * (LogDistance propagation). Multi-hop is done at L3 (see InstallInternetStack).
 * If `pcapPrefix` is non-empty, also enables pcap capture on every radio.
 */
MeshTopology BuildGridAdhocAxTopology(uint32_t gridWidth,
                                      uint32_t gridHeight,
                                      double spacing,
                                      uint32_t chWidth = 80,
                                      const std::string& pcapPrefix = "");

#endif // TOPOLOGY_H
