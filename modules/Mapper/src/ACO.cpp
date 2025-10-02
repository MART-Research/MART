
#include "../include/ACO.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <limits>

// Objective function: communication cost plus load imbalance penalty.
double ACO::objective(const std::vector<int>& mapping, const Topology& topology) {
    double cost = 0.0;
    int n = static_cast<int>(mapping.size());
    int num_phys_nodes = topology.get_num_nodes();

    // Communication cost: sum over all pairs of virtual nodes.
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double comm_cost = comm_matrix_.empty() ? 0.0 : comm_matrix_[i][j];
            PhysicalNode a = topology.get_map_node(mapping[i]);
            PhysicalNode b = topology.get_map_node(mapping[j]);
            int dist = topology.distance(a, b);
            cost += comm_cost * dist;
        }
    }

    // Load imbalance penalty.
    std::vector<int> load(num_phys_nodes, 0);
    for (int phys : mapping)
        ++load[phys];
    double avg_load = static_cast<double>(n) / num_phys_nodes;
    double imbalance_penalty = 0.0;
    for (int l : load) {
        double diff = l - avg_load;
        imbalance_penalty += diff * diff;
    }
    cost += overload_penalty_ * imbalance_penalty;
    return cost;
}

std::vector<int> ACO::map(const std::vector<VirtualNode>& vnodes,
                          const Topology& topology) {
    int n = static_cast<int>(vnodes.size());
    int num_phys_nodes = topology.get_num_nodes();

    if (n == 0 || num_phys_nodes == 0)
        return {};

    // Initialize pheromone matrix: size = n x num_phys_nodes.
    std::vector<std::vector<double>> pheromone(n, std::vector<double>(num_phys_nodes, 1.0));
    // Heuristic information: simple (1.0), but keep matrix for extensibility.
    std::vector<std::vector<double>> heuristic(n, std::vector<double>(num_phys_nodes, 1.0));

    // Choose RNG: deterministic (seeded) or random device.
    std::mt19937 gen;
    if (deterministic_) {
        gen.seed(seed_);
    } else {
        std::random_device rd;
        gen.seed(rd());
    }
    std::uniform_real_distribution<double> uni01(0.0, 1.0);

    std::vector<int> global_best_mapping(n, 0);
    double global_best_cost = std::numeric_limits<double>::infinity();

    // Main ACO loop.
    for (int iter = 0; iter < iterations_; ++iter) {
        std::vector<std::vector<int>> ant_solutions(num_ants_, std::vector<int>(n));
        std::vector<double> ant_costs(num_ants_, 0.0);

        // For each ant, construct a solution.
        for (int ant = 0; ant < num_ants_; ++ant) {
            for (int i = 0; i < n; ++i) {
                // Calculate denominator sum for probabilities.
                double denom = 0.0;
                for (int j = 0; j < num_phys_nodes; ++j) {
                    denom += std::pow(pheromone[i][j], alpha_) * std::pow(heuristic[i][j], beta_);
                }
                if (denom <= 0.0) {
                    ant_solutions[ant][i] = (i + ant) % num_phys_nodes;
                    continue;
                }
                // Compute cumulative distribution and draw sample.
                double p = uni01(gen) * denom;
                double cumulative = 0.0;
                int chosen = 0;
                for (int j = 0; j < num_phys_nodes; ++j) {
                    cumulative += std::pow(pheromone[i][j], alpha_) * std::pow(heuristic[i][j], beta_);
                    if (cumulative >= p) {
                        chosen = j;
                        break;
                    }
                }
                ant_solutions[ant][i] = chosen;
            }
            // Evaluate the candidate solution.
            ant_costs[ant] = objective(ant_solutions[ant], topology);
            if (ant_costs[ant] < global_best_cost) {
                global_best_cost = ant_costs[ant];
                global_best_mapping = ant_solutions[ant];
            }
        }

        // Update pheromones (evaporation + deposit proportional to inverse cost).
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < num_phys_nodes; ++j) {
                pheromone[i][j] *= (1.0 - evaporation_rate_);
                if (pheromone[i][j] < 1e-6)
                    pheromone[i][j] = 1e-6;
            }
        }
        for (int ant = 0; ant < num_ants_; ++ant) {
            double deposit = 1.0 / (ant_costs[ant] + 1e-9);
            for (int i = 0; i < n; ++i) {
                int phys = ant_solutions[ant][i];
                pheromone[i][phys] += deposit;
            }
        }

        // Optional: small deterministic perturbation to pheromones to avoid ties (only in deterministic mode).
        if (deterministic_) {
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < num_phys_nodes; ++j) {
                    double noise = std::nextafter(0.0, 1.0) * (uni01(gen) * 1e-3);
                    pheromone[i][j] += noise;
                }
            }
        }
    }

    // Fallback: if never updated global_best_mapping, use balanced placement.
    if (global_best_cost == std::numeric_limits<double>::infinity()) {
        for (int i = 0; i < n; ++i)
            global_best_mapping[i] = i % num_phys_nodes;
    }

    return global_best_mapping;
}
