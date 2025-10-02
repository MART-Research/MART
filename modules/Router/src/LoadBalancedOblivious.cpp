#include "../include/LoadBalancedOblivious.hpp"
LoadBalancedOblivious::LoadBalancedOblivious(Topology *topo) : Router(topo) {}

std::vector<int> LoadBalancedOblivious::compute_path(int from, int to) const
{
    std::vector<int> path;
    int current = from;

    while (current != to)
    {
        for (int d = 0; d < 32; ++d)
        { // max is 32 dimensions
            int bit_mask = 1 << d;
            if ((current & bit_mask) != (to & bit_mask))
            {
                current ^= bit_mask;
                path.push_back(current);
            }
        }
    }

    return path;
}

std::vector<int> LoadBalancedOblivious::route_hypercube(int source, int destination, int &hopcount, int vc, int depth, double flow)
{
    validate_node(source);
    validate_node(destination);

    int src = source;
    int dest = destination;
    // int IntermediateNode = get_Random_Node(source, destination);
    // make intermediate node a random number from a set of numbers from 0 to 2^n
    int n = depth;
    int max_node = (1 << n) - 1;
    int IntermediateNode = rand() % (max_node + 1);
    while (IntermediateNode == source || IntermediateNode == destination)
    {
        IntermediateNode = rand() % (max_node + 1);
    }
    // int IntermediateNode=8;
    std::cout << "Intermediate Node: " << IntermediateNode << std::endl;
    validate_node(IntermediateNode);

    std::vector<int> path1 = compute_path(src, IntermediateNode);
    std::vector<int> path2 = compute_path(IntermediateNode, dest);

    // Combine paths first
    std::vector<int> Finalpath;
    Finalpath.insert(Finalpath.end(), path1.begin(), path1.end());
    Finalpath.insert(Finalpath.end(), path2.begin(), path2.end());

    // Apply the FULL flow to the complete end-to-end path (this represents the actual data flow)
    int current_node = src;
    for (size_t i = 0; i < Finalpath.size(); ++i)
    {
        int next_node = Finalpath[i];
        Link *link = topology->get_link(current_node, next_node);
        if (!link->getActive())
        {
            return std::vector<int>();
        }
        link->update_vc_load(vc, flow);
        // link->update_max_load();
        current_node = next_node;
    }
    hopcount = Finalpath.size(); // Update hopcount to the number of hops

    return Finalpath;
}

std::vector<int> LoadBalancedOblivious::route(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);
    // These Topologies are not quadrant based hence not supported
    if(topology->get_topology_name() == "Tree")
    {
        throw std::runtime_error("Tree topology is not supported by LoadBalancedOblivious routing.");
    }
    else if (topology->get_topology_name() == "Dragonfly")
    {
        throw std::runtime_error("Dragonfly topology is not supported by LoadBalancedOblivious routing.");
    }
    else if (topology->get_topology_name() == "Butterfly")
    {
        throw std::runtime_error("Butterfly topology is not supported by LoadBalancedOblivious routing.");
    }
    try
    {
        // Check if there is a direct link between source and destination then avoid routing
        // as we get intermediate node in quadrant hence here the intermediate node is not used
        // as it will be the destination
        if (topology->get_link(source, destination))
        {
            if (!topology->get_link(source, destination)->getActive())
            {
                // Indicate a routing fault by returning an empty path
                return std::vector<int>();
            }
            std::vector<int> path;
            path.push_back(source);
            path.push_back(destination);
            hopcount = 1;
            Link *link = topology->get_link(source, destination);
            link->update_vc_load(vc, flow);
            // link->update_max_load();
            return path;
        }
    }
    catch (const std::out_of_range &e)
    {
        // If direct link is not found, continue with the rest of the routing logic
    }
    int depths = topology->get_depth();
    if (topology->get_topology_name() == "Hypercube")
    {
        std::vector<int> check_path = route_hypercube(source, destination, hopcount, vc, depths, flow);
        for (int i = 0; i < check_path.size() - 1; i++)
        {
            Link *link = topology->get_link(check_path[i], check_path[i + 1]);
            if (!link->getActive())
            {
                // Indicate a routing fault by returning an empty path
                return std::vector<int>();
            }
        }
        return check_path;
    }
    int cols = topology->get_num_cols();

    if (topology->get_topology_name() == "Mesh" || topology->get_topology_name() == "Torus_2D" || topology->get_topology_name() == "Ring")
    {
        depths = 1; // For 2D topologies, depth is set to 1
    }
    // Calculate source and destination coordinates
    int src_row = source / (cols * depths);
    int src_col = (source / depths) % cols;
    int src_depth = source % depths;
    int dest_row = destination / (cols * depths);
    int dest_col = (destination / depths) % cols;
    int dest_depth = destination % depths;

    // Get a random intermediate node within the specified quadrant
    std::vector<int> src_coords = {src_row, src_col, src_depth};
    std::vector<int> dest_coords = {dest_row, dest_col, dest_depth};

    int IntermediateNode = topology->get_Random_Node_In_Quadrant(src_coords, dest_coords);
    while (IntermediateNode == source || IntermediateNode == destination)
    {
        IntermediateNode = topology->get_Random_Node_In_Quadrant(src_coords, dest_coords);
    }
    std::cout << "Intermediate Node: " << IntermediateNode << std::endl;
    validate_node(IntermediateNode);
    DimensionOrder dor(topology);
    std::vector<int> path1, path2, Finalpath;
    
    // First get paths without applying load
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
    
    // Combine paths first
    Finalpath.insert(Finalpath.end(), path1.begin(), path1.end());
    Finalpath.insert(Finalpath.end(), path2.begin()+1, path2.end());
    
    // Apply the FULL flow to the complete end-to-end path (this represents the actual data flow)
    for (int i = 0; i < Finalpath.size() - 1; i++) {
        Link* link = topology->get_link(Finalpath[i], Finalpath[i+1]);
        if (link) {
            link->update_vc_load(vc, flow);
            // link->update_max_load();
        }
    }
 
    hopcount = Finalpath.size() - 1; // Update hopcount to the number of hops
    return Finalpath;
}