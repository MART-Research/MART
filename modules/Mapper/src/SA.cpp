#include "../include/SA.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>

double SA::objective(const std::vector<int>& mapping, const Topology& topology) {
    double cost = 0.0;
    int n = mapping.size();
    int num_phys_nodes = topology.get_num_nodes();

    // Communication cost: sum over all pairs of virtual nodes.
    for (int i = 0; i < n; ++i) {
        for (int j = i+1; j < n; ++j) {
            double comm_cost = comm_matrix_[i][j];  // Communication cost between virtual nodes i and j.
            PhysicalNode a = topology.get_map_node(mapping[i]);
            PhysicalNode b = topology.get_map_node(mapping[j]);
            int distance = topology.distance(a, b);
            cost += comm_cost * distance;
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

std::vector<int> SA::map(const std::vector<VirtualNode>& vnodes,
                                                         const Topology& topology) {
    int n = vnodes.size();
    int num_phys_nodes = topology.get_num_nodes();

    // Initial mapping: assign each virtual node a random physical node.
    std::vector<int> current_mapping(n);
    std::mt19937 gen;
    if (deterministic_) {
        gen.seed(seed_);
    } else {
        std::random_device rd;
        gen.seed(rd());
    }
    std::uniform_int_distribution<> phys_distr(0, num_phys_nodes - 1);
    for (int i = 0; i < n; ++i) {
        current_mapping[i] = phys_distr(gen);
    }

    std::uniform_int_distribution<> vnode_distr(0, n - 1);
    std::uniform_int_distribution<> phys_change_distr(0, num_phys_nodes - 1);
    std::uniform_real_distribution<> real_distr(0.0, 1.0);

    double current_cost = objective(current_mapping, topology);
    double temperature = initial_temp_;

    std::vector<int> best_mapping = current_mapping;
    double best_cost = current_cost;

    for (int iter = 0; iter < iterations_; ++iter) {
        // Generate a neighbor: pick a random virtual node and assign a different random physical node.
        std::vector<int> new_mapping = current_mapping;
        int i = vnode_distr(gen);
        int new_assignment = phys_change_distr(gen);
        while (new_assignment == new_mapping[i]) {
            new_assignment = phys_change_distr(gen);
        }
        new_mapping[i] = new_assignment;

        double new_cost = objective(new_mapping, topology);
        double delta = new_cost - current_cost;

        if (delta < 0 || real_distr(gen) < std::exp(-delta / temperature)) {
            current_mapping = new_mapping;
            current_cost = new_cost;
            if (current_cost < best_cost) {
                best_mapping = current_mapping;
                best_cost = current_cost;
            }
        }
        temperature *= cooling_rate_;
    }

    return best_mapping;
}
