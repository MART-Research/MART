#include "../include/XorTag.hpp"
XORTag::XORTag(Topology *topo) : Router(topo) {}

std::vector<int> XORTag::route_butterfly(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);

    int k = topology->get_k();
    int n = topology->get_n();
    int current_node = source;
    std::vector<int> path;
    int dst_switch = destination % k;
    int switch_num = current_node % k;

    path.push_back(current_node);

    for (int stage = 0; stage < n - 1; ++stage)
    {
        int bit_pos = n - stage - 2;
        int diff = switch_num ^ dst_switch;

        // Flip the bit at position if needed
        if ((diff >> bit_pos) & 1)
        {
            switch_num ^= (1 << bit_pos);
        }

        int next_node = (stage + 1) * k + switch_num;
        // cout<<"Next node: "<<next_node<<endl;
        path.push_back(next_node);

        // Update link load
        Link *link = topology->get_link(current_node, next_node);
        if (!link->getActive())
        {
            // Indicate a routing fault by returning an empty path
            return std::vector<int>();
        }
        link->update_vc_load(vc, flow);
        // link->update_max_load();
        current_node = next_node;
        if (current_node == destination)
        {
            break;
        }
    }

    // No need to add destination again if it coincided to be the final node
    if (current_node != destination)
        path.push_back(destination);

    hopcount = path.size() - 1; // Update hopcount to the number of hops
    return path;
}

// Implementing the XOR-tag routing algorithm to return the entire path
std::vector<int> XORTag::route(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);
        if(topology->get_topology_name() == "Tree")
    {
        throw std::runtime_error("Tree topology is not supported by Xor Tag routing.");
    }
    else if (topology->get_topology_name() == "Dragonfly")
    {
        throw std::runtime_error("Dragonfly topology is not supported by Xor Tag routing.");
    }
    else if (topology->get_topology_name() == "Mesh")
    {
        throw std::runtime_error("Mesh topology is not supported by Xor Tag routing.");
    }
    else if (topology->get_topology_name() == "Torus_2D")
    {
        throw std::runtime_error("Torus_2D topology is not supported by Xor Tag routing.");
    }
    else if (topology->get_topology_name() == "Torus_3D")
    {
        throw std::runtime_error("Torus_3D topology is not supported by Xor Tag routing.");
    }
    else if (topology->get_topology_name() == "Ring")
    {
        throw std::runtime_error("Ring topology is not supported by Xor Tag routing.");
    }
    if (topology->get_topology_name() == "Butterfly")
    {
        return route_butterfly(source, destination, hopcount, vc, flow);
    }
    int current_node = source;
    std::vector<int> path;
    int xor_tag = source ^ destination;
    path.push_back(current_node);
    while (current_node != destination)
    {
        int next_node = -1;

        for (int i = 0; i < sizeof(int) * 8; ++i)
        {
            if (xor_tag & (1 << i))
            {
                next_node = current_node ^ (1 << i);
                break;
            }
        }
        if (next_node == -1)
        {
            throw std::runtime_error("No valid next node found.");
        }
        Link *link = topology->get_link(current_node, next_node);
        if (!link->getActive())
        {
            // Indicate a link fault by returning an empty path
            return std::vector<int>();
        }
        link->update_vc_load(vc, flow);
        // link->update_max_load();
        current_node = next_node;
        path.push_back(current_node);
        xor_tag = current_node ^ destination;
    }
    if(path.front() != source)
    {
        path.insert(path.begin(), source);
    }
    hopcount = path.size() - 1;

    return path;
}