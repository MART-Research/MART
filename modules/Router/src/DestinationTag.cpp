#include "../include/DestinationTag.hpp"
DestinationTag::DestinationTag(Topology *topo) : Router(topo), max_load(0.0) {}
std::vector<int> DestinationTag::route_butterfly(int source, int destination, int &hopcount, int vc, double flow, int node)
{
    validate_node(source);
    validate_node(destination);
    // std::cout << "Routing from Node " << source << " to Node " << destination << ":\n";

    if (source == destination)
    {
        std::vector<int> path;
        path.push_back(source);
        return path;
    }
    std::vector<int> first_path;
    std::unordered_set<int> visited;
    int temp_hopcount = 0;
    find_first_path(source, destination, first_path, visited, temp_hopcount, vc, flow, node);
    hopcount = first_path.empty() ? 0 : first_path.size() - 1;
    return first_path;
}
// Recursive function to find the first path using DFS for Butterfly routing
void DestinationTag::find_first_path(int current_node, int destination, std::vector<int> &current_path,
                                     std::unordered_set<int> &visited, int &temp_hopcount, int vc, double flow, int node)
{
    current_path.push_back(current_node);
    visited.insert(current_node);

    if (current_node == destination)
    {
        return;
    }

    std::vector<int> neighbors = topology->get_nodes_list()[current_node]->get_neighbors();
    for (int neighbor : neighbors)
    {
        if (visited.find(neighbor) == visited.end())
        {
            temp_hopcount++;
            find_first_path(neighbor, destination, current_path, visited, temp_hopcount, vc, flow, node);
            if (!current_path.empty() && current_path.back() == destination)
            {
                return;
            }

            temp_hopcount--;
        }
    }

    // Backtrack
    visited.erase(current_node);
    current_path.pop_back();
}
std::vector<int> DestinationTag::route_hypercube(int source, int destination, int &hopcount, int vc, double flow, Tag tag)
{
    validate_node(source);
    validate_node(destination);

    std::vector<int> path;
    int current = source;
    hopcount = 0;
    int dim = topology->get_depth();
    bool is_first_hop = true; // Flag to track the first hop

    while (current != destination)
    {
        for (int i = 0; i < dim; ++i)
        {
            int move_bit = 1 << i;
            if ((current & move_bit) != (destination & move_bit))
            {
                int next_node = current ^ move_bit; // Flip the bit to make a move

                // Apply the tag check only for the first hop
                if (is_first_hop)
                {
                    if (tag.type != Tag::BITWISE || !(tag.data[0] & move_bit))
                    {
                        throw std::runtime_error("Invalid tag type or tag data for Hypercube routing.");
                    }
                    is_first_hop = false;
                }

                Link *link = topology->get_link(current, next_node);
                if (!link->getActive())
                {
                    
                    return std::vector<int>();
                }
                // Update link metrics
                link->update_vc_load(vc, flow);
                // link->update_max_load();
                path.push_back(current);
                current = next_node;
                break;
            }
        }
    }

    path.push_back(destination);
    // path.erase(path.begin());
    hopcount = path.size() - 1; // Update hopcount
    return path;
}

std::vector<int> DestinationTag::route_tree(int source, int destination, int &hopcount, int/* vc*/, double /*flow*/)
{
    validate_node(source);
    validate_node(destination);

    const Tree *tree = dynamic_cast<const Tree *>(topology);
    std::vector<int> path;
    if (source == destination)
    {
        return path;
    }

    // Perform BFS to find the shortest path from source to destination
    std::unordered_map<int, int> parent;
    std::queue<int> bfsqueue;
    std::unordered_set<int> visited;

    bfsqueue.push(source);
    visited.insert(source);
    parent[source] = -1;

    while (!bfsqueue.empty())
    {
        int cur = bfsqueue.front();
        bfsqueue.pop();
        if (cur == destination)
        {
            break;
        }

        for (int neighbor : tree->get_node_relations(cur))
        {
            if (visited.find(neighbor) == visited.end())
            {
                visited.insert(neighbor);
                parent[neighbor] = cur;
                bfsqueue.push(neighbor);
            }
        }
    }

    int current = destination;
    while (current != -1)
    {
        path.push_back(current);
        current = parent[current];
    }

    std::reverse(path.begin(), path.end()); // Reverse to get the correct order
    hopcount = path.size() - 1;
    return path;
}
std::vector<int> DestinationTag::route_dragonfly(int source, int destination, int &hopcount, int vc, double flow, Tag tag)
{
    int rtr_per_group = topology->get_routers_per_group();

    int src_router = topology->get_nodes_list()[source]->get_router_id();
    int dest_group = tag.data[0];
    int dest_router = tag.data[1];
    int src_group = topology->get_nodes_list()[source]->get_group();
    std::vector<int> path;

    Dragonfly *dfly = dynamic_cast<Dragonfly *>(topology);
    if (!dfly)
    {
        throw std::runtime_error("Topology is not Dragonfly");
    }

    // //Add the source router first
    // path.push_back(src_router);

    if (src_group == dest_group)
    {
        std::cout << "Intragroup routing within Group " << src_group << "\n";
        if (src_router != dest_router)
        {
            std::cout << "Passing through Router: " << dest_router << "\n";
            path.push_back(src_router);
            path.push_back(dest_router);

            Link *link = topology->get_link(src_router, dest_router);
            if (link)
            {
                if (link->getActive() == false)
                {
                    cerr << "FAIlED: Link is inactive between " << src_router << " and " << dest_router << "\n";
                    return std::vector<int>();
                }

                link->update_vc_load(vc, flow);
                // link->update_max_load();
            }
            else
            {
                throw std::runtime_error("Routing error: link not found between " + std::to_string(src_router) + " and " + std::to_string(dest_router));
            }
        }
        else
        {
            std::cout << "Source and Destination Routers are the same: " << src_router << "\n";
            path.push_back(src_router);
            // hopcount++;
        }
    }
    else
    {
        std::cout << "Intergroup routing from Group " << src_group << " to Group " << dest_group << "\n";
        // Look through the routers neighbors, not nodes
        auto routers = dfly->get_routers();
        Router_node *src_rtr_ptr = routers[src_router];

        int intermediate_router = -1;
        for (const auto &neighbor_name : src_rtr_ptr->get_global_neighbors())
        {
            if (neighbor_name[0] == 'R')
            {
                int neighbor_id = std::stoi(neighbor_name.substr(1));
                if ((neighbor_id / rtr_per_group) == dest_group)
                {
                    intermediate_router = neighbor_id;
                    break;
                }
            }
        }

        if (intermediate_router == -1)
            throw std::runtime_error("No global link found!");

        std::cout << "Passing through Intermediate Router: " << intermediate_router << "\n";
        path.push_back(src_router);
        // hopcount++;
        path.push_back(intermediate_router);
        // hopcount++;

        Link *link = topology->get_link(src_router, intermediate_router);
        if (link)
        {
            if (link->getActive() == false)
            {
                // Indicate a routing fault by returning an empty path
                cerr<<"FAILED: Link is inactive between " << src_router << " and " << intermediate_router << "\n";
                return std::vector<int>();
            }
            link->update_vc_load(vc, flow);
            // link->update_max_load();
        }
        else
        {
            throw std::runtime_error("Routing error: link not found between " + std::to_string(src_router) + " and " + std::to_string(intermediate_router));
        }

        if (intermediate_router != dest_router)
        {
            std::cout << "Passing through Router: " << dest_router << "\n";
            path.push_back(dest_router);
            // hopcount++;

            Link *next_link = topology->get_link(intermediate_router, dest_router);
            if (next_link)
            {
                if (next_link->getActive() == false)
                {
                    cerr<<"Failure: Link is inactive between " << intermediate_router << " and " << dest_router << "\n";
                    return std::vector<int>();
                }
            }
            else
            {
                throw std::runtime_error("Routing error: link not found between " + std::to_string(intermediate_router) + " and " + std::to_string(dest_router));
            }
            if (next_link)
            {
                next_link->update_vc_load(vc, flow);
                // next_link->update_max_load();
            }
        }
    }
    if (path.back() != destination)
    {
        path.push_back(destination);
    }
    if(path.front()!= source)
    {
        path.insert(path.begin(), source);
    }
    hopcount = path.size() - 1;
    return path;
}
// Helper method to generate a tag based on topology
Tag DestinationTag::generate_tag(int source, int destination)
{
    std::string topo_name = topology->get_topology_name();

    if (topo_name == "Hypercube")
    {
        // Bitwise tag: XOR to find differing dimensions
        int tag_value = source ^ destination;
        return Tag(Tag::BITWISE, {tag_value});
    }
    else if (topo_name == "Mesh" || topo_name == "Torus_2D" || topo_name == "Torus_3D" || topo_name == "Ring") // add ring
    {
        // Coordinate-based tag: [row, col, depth]
        int cols = topology->get_num_cols();
        int depths = (topo_name == "Torus_3D") ? topology->get_depth() : 1;

        int dest_row = destination / (cols * depths);
        int dest_col = (destination / depths) % cols;
        int dest_depth = destination % depths;

        return Tag(Tag::COORDINATE, {dest_row, dest_col, dest_depth});
    }
    else if (topo_name == "Dragonfly")
    {
        // Hierarchical tag: [group, router]
        int dest_group = topology->get_nodes_list()[destination]->get_group();
        int dest_router = topology->get_nodes_list()[destination]->get_router_id();
        return Tag(Tag::HIERARCHICAL, {dest_group, dest_router});
    }
    else if (topo_name == "Tree" || topo_name == "Butterfly")
    {
        // Path-encoded tag: precompute path   // here the tag is the path itself
        std::vector<int> path;
        int dummy_hopcount = 0;
        if (topo_name == "Tree")
        {
            path = route_tree(source, destination, dummy_hopcount, 0, 0.0);
        }
        else
        {
            path = route_butterfly(source, destination, dummy_hopcount, 0, 0.0);
        }
        return Tag(Tag::PATH_ENCODED, path);
    }
    throw std::runtime_error("Unsupported topology for tag generation.");
}
std::vector<int> DestinationTag::route_mesh_torus_ring(int source, int destination, int &hopcount, int vc, double flow, Tag tag)
{
    validate_node(source);
    validate_node(destination);

    if (tag.type != Tag::COORDINATE || tag.data.size() < 2)
    {
        throw std::runtime_error("Invalid tag type for Mesh/Torus/Ring routing");
    }

    int rows = topology->get_num_rows();
    int cols = topology->get_num_cols();
    int depths = (topology->get_topology_name() == "Torus_3D") ? topology->get_depth() : 1;

    // Extract destination coordinates from the tag
    int dest_row = tag.data[0];
    int dest_col = tag.data[1];
    int dest_depth = (tag.data.size() > 2) ? tag.data[2] : 0;

    // Calculate source coordinates
    int src_row = source / (cols * depths);
    int src_col = (source / depths) % cols;
    int src_depth = source % depths;

    std::vector<int> path;
    int current_node = source;
    path.push_back(current_node);
    if (current_node == destination)
        return path;
    while (current_node != destination)
    {
        int next_node = -1;

        // Routing logic for Mesh, Torus_2D, Torus_3D, and Ring
        if (topology->get_topology_name() == "Mesh")
        {
            // Move in X direction first, then Y
            if (src_col != dest_col)
            {
                next_node = (src_col < dest_col) ? current_node + 1 : current_node - 1;
            }
            else if (src_row != dest_row)
            {
                next_node = (src_row < dest_row) ? current_node + cols : current_node - cols;
            }
        }
        else if (topology->get_topology_name() == "Torus_2D" || topology->get_topology_name() == "Ring")
        {
            // Handle wrap-around in X and Y directions
            if (src_col != dest_col)
            {
                int direct_dist_x = std::abs(src_col - dest_col);
                int wrap_dist_x = cols - direct_dist_x;

                if (src_col < dest_col)
                {
                    src_col = (direct_dist_x <= wrap_dist_x) ? src_col + 1 : src_col - 1;
                    if (src_col < 0)
                        src_col += cols;
                }
                else
                {
                    src_col = (direct_dist_x <= wrap_dist_x) ? src_col - 1 : src_col + 1;
                    if (src_col >= cols)
                        src_col -= cols;
                }
                next_node = src_row * cols + src_col;
            }
            else if (src_row != dest_row)
            {
                int direct_dist_y = std::abs(src_row - dest_row);
                int wrap_dist_y = rows - direct_dist_y;

                if (src_row < dest_row)
                {
                    src_row = (direct_dist_y <= wrap_dist_y) ? src_row + 1 : src_row - 1;
                    if (src_row < 0)
                        src_row += rows;
                }
                else
                {
                    src_row = (direct_dist_y <= wrap_dist_y) ? src_row - 1 : src_row + 1;
                    if (src_row >= rows)
                        src_row -= rows;
                }
                next_node = src_row * cols + src_col;
            }
        }
        else if (topology->get_topology_name() == "Torus_3D")
        {
            // Handle wrap-around in X, Y, and Z directions
            if (src_col != dest_col)
            {
                int direct_dist_x = std::abs(src_col - dest_col);
                int wrap_dist_x = cols - direct_dist_x;

                if (src_col < dest_col)
                {
                    src_col = (direct_dist_x <= wrap_dist_x) ? src_col + 1 : src_col - 1;
                    if (src_col < 0)
                        src_col += cols;
                }
                else
                {
                    src_col = (direct_dist_x <= wrap_dist_x) ? src_col - 1 : src_col + 1;
                    if (src_col >= cols)
                        src_col -= cols;
                }
                next_node = src_row * cols * depths + src_col * depths + src_depth;
            }
            else if (src_row != dest_row)
            {
                int direct_dist_y = std::abs(src_row - dest_row);
                int wrap_dist_y = rows - direct_dist_y;

                if (src_row < dest_row)
                {
                    src_row = (direct_dist_y <= wrap_dist_y) ? src_row + 1 : src_row - 1;
                    if (src_row < 0)
                        src_row += rows;
                }
                else
                {
                    src_row = (direct_dist_y <= wrap_dist_y) ? src_row - 1 : src_row + 1;
                    if (src_row >= rows)
                        src_row -= rows;
                }
                next_node = src_row * cols * depths + src_col * depths + src_depth;
            }
            else if (src_depth != dest_depth)
            {
                int direct_dist_z = std::abs(src_depth - dest_depth);
                int wrap_dist_z = depths - direct_dist_z;

                if (src_depth < dest_depth)
                {
                    src_depth = (direct_dist_z <= wrap_dist_z) ? src_depth + 1 : src_depth - 1;
                    if (src_depth < 0)
                        src_depth += depths;
                }
                else
                {
                    src_depth = (direct_dist_z <= wrap_dist_z) ? src_depth - 1 : src_depth + 1;
                    if (src_depth >= depths)
                        src_depth -= depths;
                }
                next_node = src_row * cols * depths + src_col * depths + src_depth;
            }
        }

        if (next_node == -1)
        {
            throw std::runtime_error("Failed to determine the next node for routing");
        }

        // Update link loads and max load
        Link *link = topology->get_link(current_node, next_node);
        if (!link->getActive())
        {
            // Indicate a routing fault by returning an empty path
            return std::vector<int>();
        }
        link->update_vc_load(vc, flow);
        // link->update_max_load();
        // Move to the next node
        current_node = next_node;
        path.push_back(current_node);
        // Update source coordinates for the next iteration
        src_row = current_node / (cols * depths);
        src_col = (current_node / depths) % cols;
        src_depth = current_node % depths;
    }
    hopcount = path.size() - 1; // Update hopcount
    return path;
}
std::vector<int> DestinationTag::route(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);
    Tag tag = generate_tag(source, destination);
    if (topology->get_topology_name() == "Hypercube")
    {
        return route_hypercube(source, destination, hopcount, vc, flow, tag);
    }
    else if (topology->get_topology_name() == "Dragonfly")
    {
        return route_dragonfly(source, destination, hopcount, vc, flow, tag);
    }
    else if (topology->get_topology_name() == "Tree" || topology->get_topology_name() == "Butterfly")
    {
        if (tag.type == Tag::PATH_ENCODED)
        {
            std::vector<int> path = tag.data; // Use the precomputed path
            hopcount = path.size() - 1;       // Update hopcount based on the path length

            // Update VC load and max load for each link in the path
            for (size_t i = 0; i < path.size() - 1; ++i)
            {
                Link *link = topology->get_link(path[i], path[i + 1]);
                if (!link->getActive())
                {
                    // Indicate a routing fault by returning an empty path
                    return std::vector<int>();
                }
                if (link)
                {
                    link->update_vc_load(vc, flow);
                    // link->update_max_load();
                }
                else
                {
                    throw std::runtime_error("Link not found in topology.");
                }
            }

            return path;
        }
        else
            throw std::runtime_error("Invalid tag type.");
    }
    else
    {
        return route_mesh_torus_ring(source, destination, hopcount, vc, flow, tag);
    }
}