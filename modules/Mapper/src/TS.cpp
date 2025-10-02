#include "../include/TS.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

double TS::objective(const std::vector<int>& mapping, const Topology& topology) {
    double cost = 0.0;
    int n = mapping.size();
    int num_phys_nodes = topology.get_num_nodes();

    // Communication cost: sum over all pairs of virtual nodes using communication matrix weights.
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double comm_cost = comm_matrix_[i][j];  // Use weight from the communication matrix.
            PhysicalNode a = topology.get_map_node(mapping[i]);
            PhysicalNode b = topology.get_map_node(mapping[j]);
            int d = topology.distance(a, b);
            cost += comm_cost * d;
        }
    }

    // Compute load imbalance penalty.
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

std::vector<int> TS::map(const std::vector<VirtualNode>& vnodes, const Topology& topology) {
    int n = vnodes.size();
    int num_phys_nodes = topology.get_num_nodes();
    if(n == 0) return std::vector<int>();

    std::mt19937 gen;
    if (deterministic_) {
        gen.seed(seed_);
    } else {
        std::random_device rd;
        gen.seed(rd());
    }
    std::uniform_int_distribution<> phys_distr(0, num_phys_nodes - 1);
    std::uniform_int_distribution<> vnode_distr(0, n - 1);

    // Initialize the current solution with a random mapping.
    std::vector<int> current_solution(n);
    for (int i = 0; i < n; i++) {
        current_solution[i] = phys_distr(gen);
    }
    double current_cost = objective(current_solution, topology);
    
    std::vector<int> best_solution = current_solution;
    double best_cost = current_cost;

    // Create a tabu list: a 2D matrix [virtual_node][physical_assignment] storing the iteration until which that move is tabu.
    std::vector<std::vector<int>> tabu_list(n, std::vector<int>(num_phys_nodes, 0));

    // Main TS loop.
    for (int iter = 1; iter <= iterations_; iter++) {
        std::vector<int> best_candidate = current_solution;
        double best_candidate_cost = std::numeric_limits<double>::infinity();
        int best_vnode = -1;
        int best_new_assignment = -1;

        // Consider a subset of neighborhood moves.
        for (int k = 0; k < neighborhood_size_; k++) {
            int vnode = vnode_distr(gen);
            for (int assignment = 0; assignment < num_phys_nodes; assignment++) {
                if (assignment == current_solution[vnode])
                    continue;  // No change.
                
                // Check tabu status.
                if (iter < tabu_list[vnode][assignment])
                    continue;
                
                // Create a candidate solution by changing the assignment for vnode.
                std::vector<int> candidate = current_solution;
                candidate[vnode] = assignment;
                double candidate_cost = objective(candidate, topology);
                
                // Aspiration criterion: accept move even if tabu when candidate is better than global best.
                if (candidate_cost < best_candidate_cost ||
                    (iter < tabu_list[vnode][assignment] && candidate_cost < best_cost)) {
                    best_candidate = candidate;
                    best_candidate_cost = candidate_cost;
                    best_vnode = vnode;
                    best_new_assignment = assignment;
                }
            }
        }
        
        // If no candidate found (all moves tabu), choose a random move.
        if (best_candidate_cost == std::numeric_limits<double>::infinity()) {
            int vnode = vnode_distr(gen);
            int assignment = phys_distr(gen);
            while (assignment == current_solution[vnode])
                assignment = phys_distr(gen);
            best_candidate = current_solution;
            best_candidate[vnode] = assignment;
            best_candidate_cost = objective(best_candidate, topology);
            best_vnode = vnode;
            best_new_assignment = assignment;
        }
        
        // Update current solution.
        current_solution = best_candidate;
        current_cost = best_candidate_cost;
        
        // Update the tabu list for the move: forbid reversing move for tabu_tenure_ iterations.
        if (best_vnode != -1) {
            tabu_list[best_vnode][current_solution[best_vnode]] = iter + tabu_tenure_;
        }
        
        // Update global best if improved.
        if (current_cost < best_cost) {
            best_cost = current_cost;
            best_solution = current_solution;
        }
    }
    
    return best_solution;
}
