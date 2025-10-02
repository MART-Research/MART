#ifndef PSO_MAPPING_HPP
#define PSO_MAPPING_HPP

#include "../Mapping.hpp"
#include <vector>

// PSO-based mapping algorithm.

class PSO : public MappingAlgorithm {
public:
    // deterministic: if true, use deterministic (seeded) RNG; if false, use random device.
    // seed: seed for deterministic mode (ignored if deterministic is false).
    PSO(int swarm_size = 30, int iterations = 1000,
        double w = 0.729, double c1 = 1.49445, double c2 = 1.49445,
        double overload_penalty = 10.0, bool deterministic = false, unsigned int seed = 42u)
        : swarm_size_(swarm_size), iterations_(iterations),
          w_(w), c1_(c1), c2_(c2), overload_penalty_(overload_penalty),
          deterministic_(deterministic), seed_(seed) {}

    // Set a communication matrix (assumed symmetric). This matrix is used in the objective.
    void set_comm_matrix(const std::vector<std::vector<double>>& comm_matrix) {
        comm_matrix_ = comm_matrix;
    }

    // Optionally change the seed and deterministic mode at runtime.
    void set_seed(unsigned int seed) { seed_ = seed; }
    void set_deterministic(bool deterministic) { deterministic_ = deterministic; }

    // Map virtual nodes to physical nodes.
    // This method works for many-to-one assignments.
    std::vector<int> map(const std::vector<VirtualNode>& vnodes,
                         const Topology& topology) override;

private:
    int swarm_size_;
    int iterations_;
    double w_, c1_, c2_;
    double overload_penalty_;
    bool deterministic_;
    unsigned int seed_;
    std::vector<std::vector<double>> comm_matrix_;

    // Compute the objective value for a given discrete mapping.
    double objective(const std::vector<int>& mapping, const Topology& topology);
};

#endif // PSO_MAPPING_HPP
