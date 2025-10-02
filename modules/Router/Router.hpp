#ifndef Router_HPP
#define Router_HPP

#include <vector>
#include <stdexcept>
#include "../utils/networks/Topology.hpp"

// Abstract base class for routing algorithms
class Router {
protected:
    Topology* topology;  // Pointer to the topology

public:
    // Constructor that takes a topology reference
    Router(Topology* topo) : topology(topo) {}

    // Virtual destructor
    virtual ~Router() {}

    // Pure virtual function to determine the path from source to destination
    virtual std::vector<int> route(int source, int destination,int& hopcount,int vc=1,double flow=0.0)
    {
        (void)source;  // Suppress unused parameter warning
        (void)destination;  // Suppress unused parameter warning
        (void)hopcount;  // Suppress unused parameter warning
        (void)vc;  // Suppress unused parameter warning
        (void)flow;  // Suppress unused parameter warning
        // validate_node(source);
        // validate_node(destination);
        return {};
        // Return an empty path by default
    }
    virtual double get_max_load() const
    {
        return 0.0;
    }

protected:
    // Helper method to validate nodes within the topology bounds
    void validate_node(int node) const {
        if (node < 0 || node >= topology->get_num_nodes()) {
            throw std::out_of_range("Invalid node ID for the given topology");
        }
    }
};

#endif
