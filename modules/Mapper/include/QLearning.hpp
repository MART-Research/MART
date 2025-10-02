#ifndef QLEARNING_MAPPING_HPP
#define QLEARNING_MAPPING_HPP

#include "../Mapping.hpp"
#include <vector>

// QLearning Mapping Algorithm
// For each virtual node, the algorithm chooses a physical node (action) 
// using an epsilon-greedy policy on a Q-table that estimates the (negative) incremental cost.

class QLearning : public MappingAlgorithm {
public:
    // Constructor with Q-learning parameters:
    // alpha: learning rate.
    // gamma: discount factor (set to 0.0 if decisions are independent).
    // epsilon: exploration probability.
    // deterministic: if true, use deterministic (seeded) RNG; if false, use random device.
    // seed: seed for deterministic mode (ignored if deterministic is false).
    QLearning(double alpha = 0.1, double gamma = 0.0, double epsilon = 0.2,
              bool deterministic = false, unsigned int seed = 42u)
        : alpha_(alpha), gamma_(gamma), epsilon_(epsilon), deterministic_(deterministic), seed_(seed) {}

    // Set the communication matrix.
    void set_comm_matrix(const std::vector<std::vector<double>>& comm_matrix) {
        comm_matrix_ = comm_matrix;
    }

    // Optionally change the seed and deterministic mode at runtime.
    void set_seed(unsigned int seed) { seed_ = seed; }
    void set_deterministic(bool deterministic) { deterministic_ = deterministic; }

    // Map virtual nodes to physical nodes.
    std::vector<int> map(const std::vector<VirtualNode>& vnodes,
                         const Topology& topology) override;

private:
    double alpha_;
    double gamma_;
    double epsilon_;
    bool deterministic_;
    unsigned int seed_;
    std::vector<std::vector<double>> comm_matrix_;

    // Compute incremental cost for assigning virtual node v to physical node a,
    // given the current assignment for previously processed virtual nodes.
    double incremental_cost(int v, int a, const std::vector<int>& current_mapping, const Topology& topology);
};

#endif // QLEARNING_MAPPING_HPP
