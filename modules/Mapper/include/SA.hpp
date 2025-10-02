#ifndef SA_MAPPING_HPP
#define SA_MAPPING_HPP

#include "../Mapping.hpp"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>


class SA : public MappingAlgorithm {
public:
    // Single constructor with default parameters.
    // deterministic: if true, use deterministic (seeded) RNG; if false, use random device.
    // seed: seed for deterministic mode (ignored if deterministic is false).
    SA(double initial_temp = 1000.0,
       double cooling_rate = 0.95,
       int iterations = 1000,
       double overload_penalty = 10.0,
       bool deterministic = false, unsigned int seed = 42u)
        : initial_temp_(initial_temp),
          cooling_rate_(cooling_rate),
          iterations_(iterations),
          overload_penalty_(overload_penalty),
          deterministic_(deterministic), seed_(seed) {}

    // Set the communication matrix used in the objective function.
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
    double initial_temp_;
    double cooling_rate_;
    int iterations_;
    double overload_penalty_;
    bool deterministic_;
    unsigned int seed_;
    std::vector<std::vector<double>> comm_matrix_;

    // Helper function: Manhattan distance.
    int manhattan_distance(const PhysicalNode& a, const PhysicalNode& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    // Objective function: sum of communication cost (comm_matrix * physical distance)
    // plus a load imbalance penalty.
    double objective(const std::vector<int>& mapping, const Topology& topology);
};

#endif // SA_MAPPING_HPP
