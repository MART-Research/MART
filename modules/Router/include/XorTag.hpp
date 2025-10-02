#ifndef XOR_TAG_H
#define XOR_TAG_H

#include "../Router.hpp"
#include <vector>
#include "../../utils/links/Link.hpp"

class XORTag : public Router
{
public:
    // Constructor: Passes topology reference to the base class
    XORTag(Topology *topo);

    std::vector<int> route_butterfly(int source, int destination, int &hopcount, int vc, double flow);

    std::vector<int> route(int source, int destination, int &hopcount, int vc, double flow) override;
};

#endif // XOR_TAG_ROUTER_H