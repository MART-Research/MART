#ifndef TS_MAPPING_HPP
#define TS_MAPPING_HPP

#include "../Mapping.hpp"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>


class TS : public MappingAlgorithm {
public:
    // Constructor with TS parameters:
    // iterations: total iterations to run.
    // tabu_tenure: number of iterations a move is kept tabu.
    // neighborhood_size: number of neighbors to consider in each iteration.
    // overload_penalty: penalty factor for load imbalance.
    // deterministic: if true, use deterministic (seeded) RNG; if false, use random device.
    // seed: seed for deterministic mode (ignored if deterministic is false).
    TS(int iterations = 1000, int tabu_tenure = 10, int neighborhood_size = 20, double overload_penalty = 10.0,
       bool deterministic = false, unsigned int seed = 42u)
        : iterations_(iterations), tabu_tenure_(tabu_tenure), neighborhood_size_(neighborhood_size),
          overload_penalty_(overload_penalty), deterministic_(deterministic), seed_(seed) {}

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
    int iterations_;
    int tabu_tenure_;
    int neighborhood_size_;
    double overload_penalty_;
    bool deterministic_;
    unsigned int seed_;
    std::vector<std::vector<double>> comm_matrix_;

    // Objective function: computes total cost as the sum over all pairs of virtual nodes 
    // (comm_weight from comm_matrix * physical distance) plus a load imbalance penalty.
    double objective(const std::vector<int>& mapping, const Topology& topology);
};

#endif // TS_MAPPING_HPP
