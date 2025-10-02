#include "../include/BnB.hpp"
#include <random>
#include <cmath>
#include <iostream>

// Compute the objective cost for a complete mapping.
// The cost is defined as the sum of communication cost for every pair of virtual nodes
// plus a load imbalance penalty.
double BnB::objective(const std::vector<int>& mapping, const Topology& topology) {
    double cost = 0.0;
    int n = mapping.size();
    int num_phys_nodes = topology.get_num_nodes();
    // Communication cost.
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double comm_cost = comm_matrix_[i][j];
            PhysicalNode a = topology.get_map_node(mapping[i]);
            PhysicalNode b = topology.get_map_node(mapping[j]);
            int d = topology.distance(a, b);
            cost += comm_cost * d;
        }
    }
    // Load imbalance penalty.
    std::vector<int> load(num_phys_nodes, 0);
    for (int phys : mapping) {
        load[phys]++;
    }
    double avg_load = static_cast<double>(n) / num_phys_nodes;
    double imbalance_penalty = 0.0;
    for (int l : load) {
        double diff = l - avg_load;
        imbalance_penalty += diff * diff;
    }
    cost += overload_penalty_ * imbalance_penalty;
    return cost;
}

// Recursively explore assignments of virtual nodes (0 to n-1).
// 'level' indicates the current virtual node index we are assigning.
void BnB::branch_and_bound(int level, int n, int num_phys_nodes,
                                   const Topology& topology, std::vector<int>& current) {
    // If a full mapping is reached, compute the objective cost.
    if (level == n) {
        double cost = objective(current, topology);
        if (cost < best_cost_) {
            best_cost_ = cost;
            best_mapping_ = current;
        }
        return;
    }
    
    // Compute partial cost (only for already assigned pairs).
    double partial_cost = 0.0;
    for (int i = 0; i < level; i++) {
        for (int j = i + 1; j < level; j++) {
            double comm_cost = comm_matrix_[i][j];
            PhysicalNode a = topology.get_map_node(current[i]);
            PhysicalNode b = topology.get_map_node(current[j]);
            partial_cost += comm_cost * topology.distance(a, b);
        }
    }
    
    // Simple lower bound: assume remaining assignments incur zero additional cost.
    if (partial_cost >= best_cost_) {
        // Prune this branch.
        return;
    }
    
    // For the current virtual node at 'level', try all physical nodes.
    for (int phys = 0; phys < num_phys_nodes; phys++) {
        current[level] = phys;
        branch_and_bound(level + 1, n, num_phys_nodes, topology, current);
    }
}

std::vector<int> BnB::map(const std::vector<VirtualNode>& vnodes, const Topology& topology) {
    int n = vnodes.size();
    int num_phys_nodes = topology.get_num_nodes();
    
    // Initialize best cost to a high value.
    best_cost_ = std::numeric_limits<double>::infinity();
    best_mapping_.assign(n, 0);
    
    // Start recursion with an empty assignment vector.
    std::vector<int> current(n, 0);
    branch_and_bound(0, n, num_phys_nodes, topology, current);
    
    return best_mapping_;
}
