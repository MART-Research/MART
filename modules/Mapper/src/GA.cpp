#include "../include/GA.hpp"
#include <random>
#include <algorithm>
#include <iostream>
#include <limits>
#include <cmath>

double GA::objective(const std::vector<int>& mapping, const Topology& topology) {
    double cost = 0.0;
    int n = mapping.size();
    int num_phys_nodes = topology.get_num_nodes();
    // Communication cost: sum over all pairs of virtual nodes.
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double comm_cost = comm_matrix_[i][j];
            PhysicalNode a = topology.get_map_node(mapping[i]);
            PhysicalNode b = topology.get_map_node(mapping[j]);
            int d = topology.distance(a, b);
            cost += comm_cost * d;
        }
    }
    // Load imbalance penalty.
    std::vector<int> load(num_phys_nodes, 0);
    for (int phys : mapping)
        load[phys]++;
    double avg_load = static_cast<double>(n) / num_phys_nodes;
    double imbalance_penalty = 0.0;
    for (int l : load) {
        double diff = l - avg_load;
        imbalance_penalty += diff * diff;
    }
    cost += overload_penalty_ * imbalance_penalty;
    return cost;
}

std::vector<int> GA::map(const std::vector<VirtualNode>& vnodes, const Topology& topology) {
    int n = vnodes.size();
    int num_phys_nodes = topology.get_num_nodes();
    // Initialize population: each candidate solution is a vector<int> of length n.
    std::vector<std::vector<int>> population(population_size_, std::vector<int>(n));
    std::mt19937 gen;
    if (deterministic_) {
        gen.seed(seed_);
    } else {
        std::random_device rd;
        gen.seed(rd());
    }
    std::uniform_int_distribution<> phys_distr(0, num_phys_nodes - 1);
    for (int i = 0; i < population_size_; i++) {
        for (int j = 0; j < n; j++) {
            population[i][j] = phys_distr(gen);
        }
    }
    
    // Evaluate initial fitness.
    auto fitness = [&](const std::vector<int>& mapping) {
        return objective(mapping, topology);
    };
    std::vector<double> fitnesses(population_size_);
    for (int i = 0; i < population_size_; i++) {
        fitnesses[i] = fitness(population[i]);
    }
    
    std::vector<int> best_solution = population[0];
    double best_fitness = fitnesses[0];
    for (int i = 1; i < population_size_; i++) {
        if (fitnesses[i] < best_fitness) {
            best_fitness = fitnesses[i];
            best_solution = population[i];
        }
    }
    
    // GA main loop.
    std::uniform_real_distribution<> real_dist(0.0, 1.0);
    for (int gen_idx = 0; gen_idx < generations_; gen_idx++) {
        // Selection: using tournament selection (tournament size = 2).
        std::vector<std::vector<int>> new_population;
        new_population.reserve(population_size_);
        std::uniform_int_distribution<> index_dist(0, population_size_ - 1);
        while (new_population.size() < population_size_) {
            int i1 = index_dist(gen);
            int i2 = index_dist(gen);
            int selected = (fitnesses[i1] < fitnesses[i2]) ? i1 : i2;
            new_population.push_back(population[selected]);
        }
        // Crossover: single-point crossover.
        std::uniform_int_distribution<> cross_dist(1, n - 1);
        for (int i = 0; i < population_size_ - 1; i += 2) {
            if (real_dist(gen) < crossover_rate_) {
                int cp = cross_dist(gen);
                for (int j = cp; j < n; j++) {
                    std::swap(new_population[i][j], new_population[i+1][j]);
                }
            }
        }
        // Mutation: randomly change a gene.
        for (int i = 0; i < population_size_; i++) {
            for (int j = 0; j < n; j++) {
                if (real_dist(gen) < mutation_rate_) {
                    new_population[i][j] = phys_distr(gen);
                }
            }
        }
        // Evaluate new population.
        population = new_population;
        for (int i = 0; i < population_size_; i++) {
            fitnesses[i] = fitness(population[i]);
            if (fitnesses[i] < best_fitness) {
                best_fitness = fitnesses[i];
                best_solution = population[i];
            }
        }
    }
    
    return best_solution;
}
