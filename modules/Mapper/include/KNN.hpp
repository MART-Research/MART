#ifndef KNN_MAPPING_HPP
#define KNN_MAPPING_HPP

#include "../Mapping.hpp"
#include <vector>

// KNN_Mapping uses a KNN-inspired approach to assign virtual nodes based on the weighted average
// of the physical coordinates of the k most strongly communicating (already assigned) virtual neighbors.

class KNN : public MappingAlgorithm {
public:
    // k: number of neighbors to consider.
    // deterministic: if true, use deterministic (seeded) RNG; if false, use random device.
    // seed: seed for deterministic mode (ignored if deterministic is false).
    KNN(int k = 3, bool deterministic = false, unsigned int seed = 42u)
        : k_(k), deterministic_(deterministic), seed_(seed) {}

    // Set the communication matrix.
    void set_comm_matrix(const std::vector<std::vector<double>>& comm_matrix) {
        comm_matrix_ = comm_matrix;
    }

    // Optionally change the seed and deterministic mode at runtime.
    void set_seed(unsigned int seed) { seed_ = seed; }
    void set_deterministic(bool deterministic) { deterministic_ = deterministic; }

    std::vector<int> map(const std::vector<VirtualNode>& vnodes,
                         const Topology& topology) override;

private:
    int k_;
    bool deterministic_;
    unsigned int seed_;
    std::vector<std::vector<double>> comm_matrix_;
};

#endif // KNN_MAPPING_HPP
