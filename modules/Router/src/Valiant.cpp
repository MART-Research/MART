#include "../include/Valiant.hpp"
Valiant::Valiant(Topology *topo, bool xy_first) : Router(topo), xy_first(xy_first) {}
std::vector<int> Valiant::route_hypercube(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);
    srand(static_cast<unsigned int>(time(nullptr)));
    int depth = topology->get_depth(); // depth == dimension in hypercube
    if (depth < 1)
    {
        throw std::invalid_argument("Invalid dimension for hypercube topology");
    }
    int n = depth;
    int max_node = (1 << n) - 1;

    int IntermediateNode = source;
    while (IntermediateNode == source || IntermediateNode == destination)
    {
        IntermediateNode = rand() % (max_node + 1);
    }

    std::cout << "Intermediate Node: " << IntermediateNode << std::endl;
    validate_node(IntermediateNode);
    DimensionOrder dor(topology);
    dor.set_xy_first(xy_first);
    
    // Route with 0.0 flow to avoid double load application
    int hop1 = 0, hop2 = 0;
    std::vector<int> path1 = dor.route(source, IntermediateNode, hop1, vc, 0.0);
    if (path1.empty())
    {
        return std::vector<int>(); // Return empty path, meaning a faulty link along the path has been detected
    }
    std::vector<int> path2 = dor.route(IntermediateNode, destination, hop2, vc, 0.0);
    if (path2.empty())
    {
        return std::vector<int>();
    }
    std::vector<int> Finalpath;
    Finalpath.insert(Finalpath.end(), path1.begin(), path1.end());

    // Check if the last node of path1 is the same as the first node of path2
    if (!path1.empty() && !path2.empty() && path1.back() == path2.front())
    {
        Finalpath.insert(Finalpath.end(), path2.begin() + 1, path2.end()); // Skip the first node of path2
    }
    else
    {
        Finalpath.insert(Finalpath.end(), path2.begin(), path2.end());
    }
    
    // Apply the actual flow load to the complete end-to-end path (only once!)
    for (int i = 0; i < Finalpath.size() - 1; i++)
    {
        Link* link = topology->get_link(Finalpath[i], Finalpath[i+1]);
        if (link)
        {
            link->update_vc_load(vc, flow);
        }
    }
    
    hopcount = Finalpath.size() - 1; // Update hopcount to the number of hops
    return Finalpath;
}
std::vector<int> Valiant::route_tree_butterfly(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);
    DestinationTag dest(topology);
    int intermediate_node = topology->get_Random_Node(source, destination);
    validate_node(intermediate_node);
    std::cout << "Intermediate Node: " << intermediate_node << std::endl;
    
    // Route with 0.0 flow to avoid double load application
    int hop1 = 0, hop2 = 0;
    std::vector<int> path1 = dest.route(source, intermediate_node, hop1, vc, 0.0);
    if (path1.empty())
    {
        return std::vector<int>();
    }
    std::vector<int> path2 = dest.route(intermediate_node, destination, hop2, vc, 0.0);
    if (path2.empty())
    {
        return std::vector<int>();
    }
    std::vector<int> Finalpath;
    Finalpath.insert(Finalpath.end(), path1.begin(), path1.end());
    if (!path1.empty() && !path2.empty() && path1.back() == path2.front())
    {
        Finalpath.insert(Finalpath.end(), path2.begin() + 1, path2.end()); // Skip the first node of path2
    }
    else
    {
        Finalpath.insert(Finalpath.end(), path2.begin(), path2.end());
    }
    
    // Apply the actual flow load to the complete end-to-end path (only once!)
    for (int i = 0; i < Finalpath.size() - 1; i++)
    {
        Link* link = topology->get_link(Finalpath[i], Finalpath[i+1]);
        if (link)
        {
            link->update_vc_load(vc, flow);
        }
    }
    
    hopcount = Finalpath.size() - 1; // Update hopcount to the number of hops
    return Finalpath;
}
std::vector<int> Valiant::route_dragonfly(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);
    Dragonfly *dfly = dynamic_cast<Dragonfly *>(topology);
    if (!dfly)
    {
        throw std::runtime_error("Topology is not Dragonfly");
    }
    DestinationTag dest_tag(topology);
    int intermediate_node = dfly->get_Random_Node(source, destination);
    std::cout << "Intermediate Node: " << intermediate_node << std::endl;
    validate_node(intermediate_node);
    int hop1 = 0, hop2 = 0;
    std::vector<int> path1 = dest_tag.route(source, intermediate_node, hop1, vc, flow);
    if (path1.empty())
    {
        return std::vector<int>();
    }
    std::vector<int> path2 = dest_tag.route(intermediate_node, destination, hop2, vc, flow);
    if (path2.empty())
    {
        return std::vector<int>();
    }
    std::vector<int> Finalpath;
    Finalpath.insert(Finalpath.end(), path1.begin(), path1.end());

    if (!path1.empty() && !path2.empty() && path1.back() == path2.front() && path2.size() > 1)
    {
        Finalpath.insert(Finalpath.end(), std::next(path2.begin()), path2.end());
    }
    else if (!path2.empty())
    {
        Finalpath.insert(Finalpath.end(), path2.begin(), path2.end());
    }
    hopcount = Finalpath.size() - 1; // Update hopcount to the number of hops
    return Finalpath;
}
// Implement the Valiant routing algorithm
std::vector<int> Valiant::route(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);
    std::string topology_name = topology->get_topology_name();
    if (topology_name == "Hypercube")
    {
        return route_hypercube(source, destination, hopcount, vc, flow);
    }
    else if (topology_name == "Dragonfly")
    {
        return route_dragonfly(source, destination, hopcount, vc, flow);
    }
    else if (topology_name == "Tree")
    {
        try
        {
            if (topology->get_link(source, destination))
            {
                // Direct link exists, return the direct path as there can't be an intermediate node
                // std::cout << "Direct link found between " << source << " and " << destination << ". Using direct routing." << std::endl;
                hopcount = 1; // Direct link counts as one hop
                return {source, destination};
            }
        }
        catch (const std::out_of_range &e)
        {
            // No direct link, continue with Valiant routing
        }
        return route_tree_butterfly(source, destination, hopcount, vc, flow);
    }
    else if (topology_name == "Butterfly")
    {
        return route_tree_butterfly(source, destination, hopcount, vc, flow);
    }
    else if (topology_name == "Mesh" || topology_name == "Torus_2D" || topology_name == "Ring" || topology_name == "Torus_3D")
    {
        // For Mesh, Torus, and Ring topologies, we can use DimensionOrder routing
        int IntermediateNode = topology->get_Random_Node(source, destination);
        std::cout << "Intermediate Node: " << IntermediateNode << std::endl;
        validate_node(IntermediateNode);
        DimensionOrder dor(topology);
        std::vector<int> path1, path2, Finalpath;
        dor.set_xy_first(xy_first);
        
        // Route with 0.0 flow to avoid double load application
        int hop1 = 0, hop2 = 0;
        path1 = dor.route(source, IntermediateNode, hop1, vc, 0.0);
        if (path1.empty())
        {
            return std::vector<int>();
        }
        path2 = dor.route(IntermediateNode, destination, hop2, vc, 0.0);
        if (path2.empty())
        {
            return std::vector<int>();
        }
        
        // Combine paths
        Finalpath.insert(Finalpath.end(), path1.begin(), path1.end());
        // Check if the last node of path1 is the same as the first node of path2
        if (!path1.empty() && !path2.empty() && path1.back() == path2.front())
        {
            Finalpath.insert(Finalpath.end(), path2.begin() + 1, path2.end()); // Skip the first node of path2
        }
        else
        {
            Finalpath.insert(Finalpath.end(), path2.begin(), path2.end());
        }
        
        // Apply the actual flow load to the complete end-to-end path (only once!)
        for (int i = 0; i < Finalpath.size() - 1; i++)
        {
            Link* link = topology->get_link(Finalpath[i], Finalpath[i+1]);
            if (link)
            {
                link->update_vc_load(vc, flow);
            }
        }
        
        hopcount = Finalpath.size() - 1; // Update hopcount to the number of hops
        return Finalpath;
    }
    else
    {
        throw std::runtime_error("Unsupported topology for Valiant routing: " + topology_name);
    }
    return std::vector<int>(); // Return an empty path if no valid routing is found
}