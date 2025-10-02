#ifndef MAPPING_ALGORITHMS_HPP
#define MAPPING_ALGORITHMS_HPP

#include "../utils/nodes/map_nodes.hpp"
#include "../utils/networks/Topology.hpp"
#include <vector>
#include <string>

// Abstract mapping algorithm interface.
class MappingAlgorithm {
public:
    virtual ~MappingAlgorithm() {}
    // Map virtual nodes to physical nodes.
    // Returns a vector where the index is the virtual node id and the value is the physical node id assigned.
    virtual std::vector<int> map(const std::vector<VirtualNode>& vnodes,
                                 const Topology& topology) = 0;
};

#endif // MAPPING_ALGORITHMS_HPP
