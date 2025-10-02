#ifndef NODES_HPP
#define NODES_HPP

#include <string>

// Represents a virtual node (e.g., a task)
struct VirtualNode {
    int id;
    std::string name;
};

// Represents a physical node in a topology
struct PhysicalNode {
    int id;
    int x, y, z; // For 2D topologies, z is 0. For 3D, it’s used.
};

#endif // NODES_HPP
