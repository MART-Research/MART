#ifndef ACO_MAPPING_HPP
#define ACO_MAPPING_HPP

#include "../Mapping.hpp"
#include <vector>

// ACO-based mapping algorithm.

class ACO : public MappingAlgorithm {
public:
    // Constructor with ACO parameters:
    // num_ants: number of ants in each iteration.
    // iterations: number of iterations.
    // alpha: influence of pheromone.
    // beta: influence of heuristic (typically inverse cost).
    // evaporation_rate: pheromone evaporation rate.
    // overload_penalty: penalty for load imbalance.
    // deterministic: if true, use deterministic (seeded) RNG; if false, use random device.
    // seed: seed for deterministic mode (ignored if deterministic is false).
    ACO(int num_ants = 30, int iterations = 1000, double alpha = 1.0,
        double beta = 2.0, double evaporation_rate = 0.5, double overload_penalty = 10.0,
        bool deterministic = false, unsigned int seed = 42u)
        : num_ants_(num_ants), iterations_(iterations), alpha_(alpha), beta_(beta),
          evaporation_rate_(evaporation_rate), overload_penalty_(overload_penalty),
          deterministic_(deterministic), seed_(seed) {}

    // Set communication matrix (symmetric). comm_matrix[i][j] indicates communication cost between virtual nodes i and j.
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
    int num_ants_;
    int iterations_;
    double alpha_;
    double beta_;
    double evaporation_rate_;
    double overload_penalty_;
    bool deterministic_;
    unsigned int seed_;
    std::vector<std::vector<double>> comm_matrix_;

    // Objective function: computes the cost of a mapping.
    double objective(const std::vector<int>& mapping, const Topology& topology);
};

#endif // ACO_MAPPING_HPP
