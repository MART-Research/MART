#ifndef BNB_MAPPING_HPP
#define BNB_MAPPING_HPP

#include "../Mapping.hpp"
#include <vector>
#include <limits>

class BnB : public MappingAlgorithm {
public:
    // overload_penalty: penalty factor for load imbalance.
    BnB(double overload_penalty = 10.0)
        : overload_penalty_(overload_penalty), best_cost_(std::numeric_limits<double>::infinity()) {}

    // Set communication matrix: comm_matrix[i][j] indicates communication cost between virtual nodes i and j.
    void set_comm_matrix(const std::vector<std::vector<double>>& comm_matrix) {
        comm_matrix_ = comm_matrix;
    }

    // Map virtual nodes to physical nodes.
    std::vector<int> map(const std::vector<VirtualNode>& vnodes,
                         const Topology& topology) override;

private:
    double overload_penalty_;
    std::vector<std::vector<double>> comm_matrix_;
    double best_cost_;
    std::vector<int> best_mapping_;

    // Compute the full objective cost for a complete mapping.
    double objective(const std::vector<int>& mapping, const Topology& topology);

    // Recursively explore assignments.
    void branch_and_bound(int level, int n, int num_phys_nodes,
                          const Topology& topology, std::vector<int>& current);
};

#endif // BNB_MAPPING_HPP
