#ifndef GREEDY_MAPPING_HPP
#define GREEDY_MAPPING_HPP

#include "../Mapping.hpp"
#include <vector>

// GreedyMapping implements a simple greedy mapping algorithm.
class Greedy : public MappingAlgorithm {
public:
    // Constructor with an overload penalty factor.
    Greedy(double overload_penalty = 10.0)
        : overload_penalty_(overload_penalty) {}

    // Set communication matrix for incremental cost calculation.
    void set_comm_matrix(const std::vector<std::vector<double>>& comm_matrix) {
        comm_matrix_ = comm_matrix;
    }

    // Map virtual nodes to physical nodes.
    std::vector<int> map(const std::vector<VirtualNode>& vnodes,
                         const Topology& topology) override;

private:
    double overload_penalty_;
    std::vector<std::vector<double>> comm_matrix_;

    // Compute the incremental cost of assigning virtual node 'vnode_index'
    // to physical node 'candidate' given current partial mapping.
    double incremental_cost(const std::vector<int>& current,
                            int vnode_index,
                            int candidate,
                            const Topology& topology);
};

#endif // GREEDY_MAPPING_HPP
