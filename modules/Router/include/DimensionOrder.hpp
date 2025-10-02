#ifndef DIMENSION_ORDER_H
#define DIMENSION_ORDER_H

#include "../Router.hpp"
#include <cmath>
#include <vector>
#include "../../utils/links/Link.hpp"

class DimensionOrder : public Router
{
private:
    mutable bool xy_first;
    double max_load;

public:
    // Constructor: Passes topology reference to the base class
    DimensionOrder(Topology *topo, bool xy = true);
    void set_xy_first(bool xy_first);
    void set_max_load(double max_load);
    double get_max_load() const override;
    std::vector<int> route_hypercube(int source, int destination, int &hopcount, int vc, double flow);
    std::vector<int> route(int source, int destination, int &hopcount, int vc, double flow) override;
};

#endif // DIMENSION_ORDER_H