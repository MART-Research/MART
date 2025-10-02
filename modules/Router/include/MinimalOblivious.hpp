#ifndef MinimalOR_H
#define MinimalOR_H

#include "../Router.hpp"
#include <vector>
#include <queue>
#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include "../../utils/networks/topologies/Dragonfly.hpp"
// #include "../../utils/networks/topologies/Tree.hpp"
// #include "DimensionOrderRouter.hpp"
using namespace std;
class MinimalOblivious : public Router // Minimal Oblivious Routing
{
public:
    MinimalOblivious(Topology *topo);

    std::vector<std::vector<int>> get_All_Minimal_paths(int source, int destination, std::vector<int> &hopcounts);

    std::vector<int> route_hypercube(int source, int destination, int &hopcount, int vc, double flow);

    std::vector<int> route_dragonfly(int source, int destination, int &hopcount, int vc, double flow);

    // std::vector<int> route_tree(int source, int destination, int &hopcount, int vc, double flow);

    std::vector<int> route_butterfly(int source, int destination, int &hopcount, int vc, double flow, int node = -1);
    void backtrack(int current_node, int destination, std::vector<int> &temp_path,
                   std::vector<int> &shortest_path, std::unordered_set<int> &visited,
                   int &temp_hopcount, int vc, double flow, int &min_hops, int node = -1);

    std::vector<int> route(int source, int destination, int &hopcount, int vc, double flow) override;
    void generateMinimalPaths(int source, int destination, std::vector<std::vector<int>> &all_paths, std::vector<int> &hopcounts);

    int getNextNode(int current_node, char move, int cols, int depths);
};

#endif // MinimalOR_H