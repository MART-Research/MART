#ifndef FLOWNETWORKBUILDER_HPP
#define FLOWNETWORKBUILDER_HPP

#include <unordered_map>
#include <vector>
#include <algorithm>
#include "../networks/Topology.hpp"
#include "../networks/topologies/Mesh.hpp"
#include "../links/Link.hpp"
inline void preprocess_topology(
    Topology& topo, // this should better be a const, and the function get_all_links should be changed to const as well
    std::unordered_map<int, std::vector<int>>& flowNetwork
) {
    
    flowNetwork.clear();

    std::vector<Link*> links = topo.get_all_links();

    for (const auto& link : links) {
        if (!link) continue;

        int node1 = link->get_node1();
        int node2 = link->get_node2();

        if (node1 == node2) {
            continue; // Skip self-loops
        }

        // Insert node1 -> node2
        auto& neighbors1 = flowNetwork[node1];
        if (std::find(neighbors1.begin(), neighbors1.end(), node2) == neighbors1.end()) {
            neighbors1.push_back(node2);
        }

        // Insert node2 -> node1
        auto& neighbors2 = flowNetwork[node2];
        if (std::find(neighbors2.begin(), neighbors2.end(), node1) == neighbors2.end()) {
            neighbors2.push_back(node1);
        }
    }

    // Verification: Now print the resulting flow network
    // for (const auto& entry : flowNetwork) {
    //     int src = entry.first;
    //     const auto& destinations = entry.second;

    //     std::cout << src << ": ";
    //     for (const auto& dst : destinations) {
    //         std::cout << dst << " ";
    //     }
    //     std::cout << std::endl;
    // }
}


inline void preprocess_topology(
    Topology& topo, // this should better be a const, and the function get_all_links should be changed to const as well
    std::unordered_map<std::pair<int, int>, int, pair_hash> &flowNetwork
    ) 
    {

    flowNetwork.clear();

    std::vector<Link*> links = topo.get_all_links();

    for (const auto& link : links) {
        if (!link) continue;

        int node1 = link->get_node1();
        int node2 = link->get_node2();

        if (node1 == node2) {
            continue; // Skip self-loops
        }

        // Insert node1 -> node2
        flowNetwork[{node1, node2}] = link->get_bandwidth();
        flowNetwork[{node2, node1}] = link->get_bandwidth();
    }

    // Verification: Now print the resulting flow network
    // for (const auto& entry : flowNetwork) {
    //     int src = entry.first.first;
    //     int dst = entry.first.second;
    //     int capacity = entry.second;
    //     // std::cout << "Link from " << src << " to " << dst << " with capacity: " << capacity << std::endl;
    // }
}

#endif // FLOWNETWORKBUILDER_HPP