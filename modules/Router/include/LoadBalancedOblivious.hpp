#include <iostream>
#include <vector>
#include "../Router.hpp"
#include "DimensionOrder.hpp"
#include "DestinationTag.hpp"
class LoadBalancedOblivious : public Router
{
public:
    LoadBalancedOblivious(Topology *topo);

    // Override get_max_load to return the global max load from topology
    double get_max_load() const override {
        double max_load = 0.0;
        const std::vector<Link *> &links = topology->get_all_links();
        for (Link *link : links)
        {
            if (link)
            {
                double link_load = link->get_historical_max_load();
                if (link_load > max_load)
                {
                    max_load = link_load;
                }
            }
        }
        return max_load;
    }

    std::vector<int> compute_path(int from, int to) const;

    std::vector<int> route_hypercube(int source, int destination, int &hopcount, int vc, int depth, double flow);
    
    std::vector<int> route_tree(int source, int destination, int &hopcount, int vc, double flow);
    
    std::vector<int> route_mesh_load_balanced(int source, int destination, int &hopcount, int vc, double flow);
    
    std::vector<int> route_dragonfly(int source, int destination, int &hopcount, int vc, double flow);
    
    std::vector<int> route_butterfly(int source, int destination, int &hopcount, int vc, double flow);

    std::vector<int> route(int source, int destination, int &hopcount, int vc, double flow) override;
};