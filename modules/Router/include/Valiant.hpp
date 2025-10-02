#ifndef Valiant_H
#define Valiant_H
#include "../Router.hpp"
#include <vector>
#include <iostream>
#include "DimensionOrder.hpp"
#include "DestinationTag.hpp"
#include "../../utils/networks/topologies/Dragonfly.hpp"

class Valiant : public Router
{
private:
    mutable bool xy_first; // Flag to determine the order of XY and YX routing
public:
    // Constructor: Passes topology reference to the base class
    Valiant(Topology *topo, bool xy_first = true);
    std::vector<int> route_hypercube(int source, int destination, int &hopcount, int vc, double flow);
    std::vector<int> route_dragonfly(int source, int destination, int &hopcount, int vc, double flow);
    std::vector<int> route_tree_butterfly(int source, int destination, int &hopcount, int vc, double flow) ;
    //contains mesh,torus,ring
    std::vector<int> route(int source, int destination, int &hopcount, int vc, double flow) override ; 
};
#endif // Valiant_H