#ifndef DESTINATION_TAG_H
#define DESTINATION_TAG_H

#include "../Router.hpp"
#include <vector>
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <climits>
#include "../../utils/links/Link.hpp"
#include "../../utils/networks/topologies/Tree.hpp"
#include "../../utils/networks/topologies/Dragonfly.hpp"
using namespace std;
class Tag
{
public:
    enum TagType
    {
        COORDINATE,   // For Mesh/Torus (e.g., [row, col, depth])
        BITWISE,      // For Hypercube (bitmask)
        HIERARCHICAL, // For Dragonfly/Fat-Tree (e.g., [group, router, node])
        PATH_ENCODED  // Precomputed path (e.g., [R1, R2, R3])
    };

    TagType type;
    std::vector<int> data; // Tag payload

    Tag(TagType t, const std::vector<int> &d) : type(t), data(d) {}
};
class DestinationTag : public Router
{
private:
    double max_load;

public:
    // Constructor: Passes topology reference to the base class
    DestinationTag(Topology *topo);
    // Destructor
    ~DestinationTag() {}
    std::vector<int> route_butterfly(int source, int destination, int &hopcount, int vc, double flow, int node = -1);

    void find_first_path(int current_node, int destination, std::vector<int> &current_path,
                         std::unordered_set<int> &visited, int &temp_hopcount, int vc, double flow, int node = -1);
    std::vector<int> route_hypercube(int source, int destination, int &hopcount, int vc, double flow, Tag tag);

    std::vector<int> route_tree(int source, int destination, int &hopcount, int vc, double flow);
    std::vector<int> route_dragonfly(int source, int destination, int &hopcount, int vc, double flow, Tag tag);
    // Helper method to generate a tag based on topology
    Tag generate_tag(int source, int destination);

    std::vector<int> route_mesh_torus_ring(int source, int destination, int &hopcount, int vc, double flow, Tag tag);
    std::vector<int> route(int source, int destination, int &hopcount, int vc, double flow) override;

};
#endif // DESTINATION_TAG_ROUTER_H
       // #endif // DESTINATION_TAG_ROUTER_H