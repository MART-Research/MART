#include "../include/QLearning.hpp"
#include <random>
#include <algorithm>
#include <limits>
#include <cmath>
#include <numeric>
#include <iostream>

double QLearning::incremental_cost(int v, int a, const std::vector<int>& current_mapping, const Topology& topology) {
    double cost = 0.0;
    // For each already assigned virtual node u (< v), accumulate communication cost.
    for (int u = 0; u < v; u++) {
        double comm = comm_matrix_[v][u];
        PhysicalNode phys_candidate = topology.get_map_node(a);
        PhysicalNode phys_assigned = topology.get_map_node(current_mapping[u]);
        cost += comm * topology.distance(phys_candidate, phys_assigned);
    }
    // Add a simple overload penalty: count current assignments for physical node 'a'
    int load = std::count(current_mapping.begin(), current_mapping.begin() + v, a);
    cost += load; // Optionally, multiply by a penalty factor.
    return cost;
}

std::vector<int> QLearning::map(const std::vector<VirtualNode>& vnodes, const Topology& topology) {
    int n = vnodes.size();
    int num_phys_nodes = topology.get_num_nodes();
    std::vector<int> mapping(n, -1);

    // Initialize the Q-table: For each virtual node, a vector of Q values for each physical node.
    std::vector<std::vector<double>> Q(n, std::vector<double>(num_phys_nodes, 0.0));

    std::mt19937 gen;
    if (deterministic_) {
        gen.seed(seed_);
    } else {
        std::random_device rd;
        gen.seed(rd());
    }
    std::uniform_real_distribution<> eps_dist(0.0, 1.0);
    std::uniform_int_distribution<> phys_dist(0, num_phys_nodes - 1);

    // Process virtual nodes sequentially
    for (int v = 0; v < n; v++) {
        int chosen_action = 0;
        // Epsilon-greedy: with probability epsilon, choose a random action.
        if (eps_dist(gen) < epsilon_) {
            chosen_action = phys_dist(gen);
        } else {
            double best_val = -std::numeric_limits<double>::infinity();
            for (int a = 0; a < num_phys_nodes; a++) {
                if (Q[v][a] > best_val) {
                    best_val = Q[v][a];
                    chosen_action = a;
                }
            }
        }
        // Compute reward as negative incremental cost.
        double cost = incremental_cost(v, chosen_action, mapping, topology);
        double reward = -cost;  // Lower cost means higher reward

        // Update Q-value for this state-action pair.
        Q[v][chosen_action] = Q[v][chosen_action] + alpha_ * (reward - Q[v][chosen_action]);
        // (Since each decision is independent, gamma is not used.)

        // Assign the action.
        mapping[v] = chosen_action;
    }
    return mapping;
}
