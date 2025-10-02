#include "../include/Greedy.hpp"
#include <limits>
#include <cmath>

// For each virtual node (vnode_index), this function computes an incremental cost if we assign it to the given candidate physical node.
// The cost is based on:
//   1. Communication cost: for each already-assigned virtual node (indices < vnode_index), add the product of 
//      the communication weight (from comm_matrix_) and the distance between the candidate and that node's assignment.
//   2. Load penalty: a simple penalty proportional to how many virtual nodes have already been assigned to the candidate.
double Greedy::incremental_cost(const std::vector<int>& current,
                                         int vnode_index,
                                         int candidate,
                                         const Topology& topology) {
    double cost = 0.0;
    
    // Communication cost: consider all previously assigned virtual nodes.
    for (int j = 0; j < vnode_index; j++) {
        double comm = comm_matrix_[vnode_index][j];
        int assigned = current[j];
        // Use the topology's distance function.
        cost += comm * topology.distance(topology.get_map_node(candidate), topology.get_map_node(assigned));
    }
    
    // Load penalty: count how many times candidate appears so far.
    int load = 0;
    for (int j = 0; j < vnode_index; j++) {
        if (current[j] == candidate)
            load++;
    }
    cost += overload_penalty_ * load;
    
    return cost;
}

std::vector<int> Greedy::map(const std::vector<VirtualNode>& vnodes,
                                    const Topology& topology) {
    int n = vnodes.size();
    int num_phys_nodes = topology.get_num_nodes();
    std::vector<int> mapping(n, -1);
    
    // Assign each virtual node in order.
    for (int i = 0; i < n; i++) {
        double best_cost = std::numeric_limits<double>::infinity();
        int best_candidate = 0;
        // Try each physical node.
        for (int candidate = 0; candidate < num_phys_nodes; candidate++) {
            double cost = incremental_cost(mapping, i, candidate, topology);
            if (cost < best_cost) {
                best_cost = cost;
                best_candidate = candidate;
            }
        }
        mapping[i] = best_candidate;
    }
    
    return mapping;
}
