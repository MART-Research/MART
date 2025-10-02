#include "../include/PSO.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>

double PSO::objective(const std::vector<int>& mapping, const Topology& topology) {
    double cost = 0.0;
    int n = mapping.size();
    int num_phys_nodes = topology.get_num_nodes();

    // Communication cost: sum over all pairs of virtual nodes.
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double comm_cost = comm_matrix_[i][j];  // Communication strength between virtual nodes i and j.
            PhysicalNode a = topology.get_map_node(mapping[i]);
            PhysicalNode b = topology.get_map_node(mapping[j]);
            int dist = topology.distance(a, b);
            cost += comm_cost * dist;
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

std::vector<int> PSO::map(const std::vector<VirtualNode>& vnodes,
                                  const Topology& topology) {
    int n = vnodes.size();
    int num_phys_nodes = topology.get_num_nodes();

    // Each particle: position vector and velocity vector (both of length n).
    struct Particle {
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<int> best_mapping;
        double best_cost;
    };

    std::mt19937 gen;
    if (deterministic_) {
        gen.seed(seed_);
    } else {
        std::random_device rd;
        gen.seed(rd());
    }
    std::uniform_real_distribution<> pos_dist(0.0, num_phys_nodes);
    std::uniform_real_distribution<> vel_dist(-1.0, 1.0);
    std::uniform_real_distribution<> r_dist(0.0, 1.0);

    std::vector<Particle> swarm(swarm_size_);
    // Initialize swarm.
    for (int p = 0; p < swarm_size_; p++) {
        swarm[p].position.resize(n);
        swarm[p].velocity.resize(n);
        for (int i = 0; i < n; i++) {
            swarm[p].position[i] = pos_dist(gen);
            swarm[p].velocity[i] = vel_dist(gen);
        }
        // Convert positions to discrete mapping.
        std::vector<int> mapping(n);
        for (int i = 0; i < n; i++) {
            int assignment = static_cast<int>(std::round(swarm[p].position[i])) % num_phys_nodes;
            if (assignment < 0) assignment += num_phys_nodes;
            mapping[i] = assignment;
        }
        double cost = objective(mapping, topology);
        swarm[p].best_mapping = mapping;
        swarm[p].best_cost = cost;
    }

    // Determine global best.
    std::vector<int> gbest = swarm[0].best_mapping;
    double gbest_cost = swarm[0].best_cost;
    for (int p = 1; p < swarm_size_; p++) {
        if (swarm[p].best_cost < gbest_cost) {
            gbest_cost = swarm[p].best_cost;
            gbest = swarm[p].best_mapping;
        }
    }

    // Main PSO loop.
    for (int iter = 0; iter < iterations_; iter++) {
        for (int p = 0; p < swarm_size_; p++) {
            for (int i = 0; i < n; i++) {
                double r1 = r_dist(gen);
                double r2 = r_dist(gen);
                // Update velocity using the difference between personal best and current position,
                // and the difference between global best and current position.
                // Note: We use the discrete best mapping values, subtracting the rounded current position.
                double diff_pbest = swarm[p].best_mapping[i] - std::round(swarm[p].position[i]);
                double diff_gbest = gbest[i] - std::round(swarm[p].position[i]);
                swarm[p].velocity[i] = w_ * swarm[p].velocity[i] + c1_ * r1 * diff_pbest + c2_ * r2 * diff_gbest;
                // Update position.
                swarm[p].position[i] += swarm[p].velocity[i];
                // Keep position in range.
                if (swarm[p].position[i] < 0) swarm[p].position[i] = 0;
                if (swarm[p].position[i] > num_phys_nodes) swarm[p].position[i] = num_phys_nodes;
            }
            // Convert new positions to a mapping.
            std::vector<int> mapping(n);
            for (int i = 0; i < n; i++) {
                int assignment = static_cast<int>(std::round(swarm[p].position[i])) % num_phys_nodes;
                if (assignment < 0) assignment += num_phys_nodes;
                mapping[i] = assignment;
            }
            double cost = objective(mapping, topology);
            // Update personal best.
            if (cost < swarm[p].best_cost) {
                swarm[p].best_cost = cost;
                swarm[p].best_mapping = mapping;
            }
            // Update global best.
            if (cost < gbest_cost) {
                gbest_cost = cost;
                gbest = mapping;
            }
        }
    }

    return gbest;
}
