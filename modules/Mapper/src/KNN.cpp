#include "../include/KNN.hpp"
#include <vector>
#include <random>
#include <limits>
#include <cmath>
#include <algorithm>
#include <numeric>

std::vector<int> KNN::map(const std::vector<VirtualNode>& vnodes,
                          const Topology& topology) {
    int n = vnodes.size();
    int num_phys_nodes = topology.get_num_nodes();
    std::vector<int> mapping(n, -1);
    
    if(n == 0) return mapping;
    
    std::mt19937 gen;
    if (deterministic_) {
        gen.seed(seed_);
    } else {
        std::random_device rd;
        gen.seed(rd());
    }
    
    // Instead of fixing the first virtual node to 0, assign first min(n, num_phys_nodes) nodes to distinct physical nodes
    int seed_count = std::min(n, num_phys_nodes);
    std::vector<int> seeds(seed_count);
    // Initialize seeds with a random permutation of [0, num_phys_nodes)
    std::iota(seeds.begin(), seeds.end(), 0);
    std::shuffle(seeds.begin(), seeds.end(), gen);
    for (int i = 0; i < seed_count; i++) {
        mapping[i] = seeds[i];
    }
    
    // Create a vector of remaining virtual node indices and shuffle them.
    std::vector<int> indices;
    for (int i = seed_count; i < n; i++) {
        indices.push_back(i);
    }
    std::shuffle(indices.begin(), indices.end(), gen);
    
    // Process each remaining virtual node in random order.
    for (int idx : indices) {
        // Collect indices of already assigned virtual nodes.
        std::vector<int> assigned;
        for (int u = 0; u < n; u++) {
            if (mapping[u] != -1)
                assigned.push_back(u);
        }
        
        // For each assigned virtual node, record its communication weight with idx.
        std::vector<std::pair<double, int>> neighbor_weights;
        for (int u : assigned) {
            double weight = comm_matrix_[idx][u];
            neighbor_weights.push_back({weight, u});
        }
        // Sort neighbors by descending communication weight.
        std::sort(neighbor_weights.begin(), neighbor_weights.end(),
                  [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
                      return a.first > b.first;
                  });
        
        // Compute weighted average of physical coordinates of the top k neighbors.
        double sum_x = 0, sum_y = 0, sum_z = 0;
        double total_weight = 0;
        int count = 0;
        const double epsilon = 1e-6;
        for (auto &pw : neighbor_weights) {
            if (count >= k_) break;
            int u = pw.second;
            double weight = pw.first + epsilon; // smoothing constant
            PhysicalNode phys = topology.get_map_node(mapping[u]);
            sum_x += weight * phys.x;
            sum_y += weight * phys.y;
            sum_z += weight * phys.z;
            total_weight += weight;
            count++;
        }
        
        // If total weight is zero, assign randomly.
        if (total_weight == 0) {
            std::uniform_int_distribution<> phys_distr(0, num_phys_nodes - 1);
            mapping[idx] = phys_distr(gen);
            continue;
        }
        
        double avg_x = sum_x / total_weight;
        double avg_y = sum_y / total_weight;
        double avg_z = sum_z / total_weight;
        
        // Optionally add a small random perturbation to the averages.
        std::uniform_real_distribution<> noise(-0.5, 0.5);
        avg_x += noise(gen);
        avg_y += noise(gen);
        avg_z += noise(gen);
        
        // Choose the physical node closest (by Euclidean distance) to the computed average.
        double best_dist = std::numeric_limits<double>::infinity();
        int best_phys = 0;
        for (int p = 0; p < num_phys_nodes; p++) {
            PhysicalNode candidate = topology.get_map_node(p);
            double dx = avg_x - candidate.x;
            double dy = avg_y - candidate.y;
            double dz = avg_z - candidate.z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < best_dist) {
                best_dist = dist;
                best_phys = p;
            }
        }
        mapping[idx] = best_phys;
    }
    
    return mapping;
}
