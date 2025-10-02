#include "../include/DimensionOrder.hpp"
DimensionOrder::DimensionOrder(Topology *topo, bool xy) : Router(topo), xy_first(xy)
{
    max_load = 0.0;
}
void DimensionOrder::set_xy_first(bool xy)
{
    this->xy_first = xy;
}
void DimensionOrder::set_max_load(double load)
{
    this->max_load = load;
}
double DimensionOrder::get_max_load() const
{
    return max_load;
}
std::vector<int> DimensionOrder::route_hypercube(int source, int destination, int &hopcount, int vc, double flow)
{
    int dimension = topology->get_depth();
    int current_node = source;
    std::vector<int> path;
    path.push_back(source);
    while (current_node != destination)
    {
        int next_node = -1;

        int differing_bit = current_node ^ destination;

        // Find the most significant differing bit
        int bit_position = dimension - 1;
        while (bit_position >= 0 && (differing_bit & (1 << bit_position)) == 0)
        {
            bit_position--;
        }

        if (bit_position >= 0)
        {
            // flip MSB to get next node
            next_node = current_node ^ (1 << bit_position);
        }

        if (next_node == -1)
        {
            throw std::runtime_error("Routing error: no valid next node found.");
        }

        Link *link = topology->get_link(current_node, next_node);
        // Check if link is active
        if (!link->getActive())
        {
            std::cout << "Routing fault: Link is inactive between " << current_node << " and " << next_node << "\n";
            return std::vector<int>();
        }
        link->update_vc_load(vc, flow);
        // link->update_max_load();
        // if (link->get_max_load() > max_load)
        // {
        //     max_load = link->get_max_load();
        // }
        current_node = next_node;
        path.push_back(current_node);
    }
    if (path.front() != source)
    {
        path.insert(path.begin(), source);
    }

    hopcount = path.size() - 1; // Update hopcount to the number of hops
    return path;
}

// Implement the dimension-order routing to return the entire path
std::vector<int> DimensionOrder::route(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);
    if (topology->get_topology_name() == "Tree")
    {
        throw std::runtime_error("Tree topology is not supported by Dimension Order routing.");
    }
    else if (topology->get_topology_name() == "Dragonfly")
    {
        throw std::runtime_error("Dragonfly topology is not supported by Dimension Order routing.");
    }
    else if (topology->get_topology_name() == "Butterfly")
    {
        throw std::runtime_error("Butterfly topology is not supported by Dimension Order routing.");
    }
    if (topology->get_topology_name() == "Hypercube")
    {
        return route_hypercube(source, destination, hopcount, vc, flow);
    }

    int rows = topology->get_num_rows();
    int cols = topology->get_num_cols();
    int depths = topology->get_depth();

    // Calculate source and destination coordinates
    int dest_row = destination / (cols * depths);
    int dest_col = (destination / depths) % cols;
    int dest_depth = destination % depths;

    int current_node = source;
    std::vector<int> path;
    path.push_back(current_node);
    // std::cout<<"called before while loop in dor::route\n";
    while (current_node != destination)
    {
        int next_node = -1;

        if (topology->get_topology_name() == "Ring" || topology->get_topology_name() == "Torus_2D" || topology->get_topology_name() == "Torus_3D")
        {
            int current_row = current_node / (cols * depths);
            int current_col = (current_node / depths) % cols;
            int current_depth = current_node % depths;

            if (xy_first)
            {
                if (current_col != dest_col)
                { // X direction
                    // Calculate direct and wrap-around distances
                    int direct_dist = std::abs(current_col - dest_col); // clockwise or anti clockwise
                    int wrap_dist = cols - direct_dist;

                    if (current_col < dest_col)
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_col++;
                        }
                        else
                        {
                            current_col--;
                            if (current_col < 0)
                                current_col += cols;
                        }
                    }
                    else
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_col--;
                        }
                        else
                        {
                            current_col++;
                            if (current_col >= cols)
                                current_col -= cols;
                        }
                    }
                    if (depths > 1)
                    {
                        next_node = current_row * cols * depths + current_col * depths + current_depth;
                    }
                    else
                    {
                        next_node = current_row * cols + current_col;
                    }
                }
                else if (current_row != dest_row)
                { // Y direction
                    // Calculate direct and wrap-around distances
                    int direct_dist = std::abs(current_row - dest_row);
                    int wrap_dist = rows - direct_dist;

                    if (current_row < dest_row)
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_row++;
                        }
                        else
                        {
                            current_row--;
                            if (current_row < 0)
                                current_row += rows;
                        }
                    }
                    else
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_row--;
                        }
                        else
                        {
                            current_row++;
                            if (current_row >= rows)
                                current_row -= rows;
                        }
                    }
                    if (depths > 1)
                    {
                        next_node = current_row * cols * depths + current_col * depths + current_depth;
                    }
                    else
                    {
                        next_node = current_row * cols + current_col;
                    }
                }
                else if (depths > 1 && current_depth != dest_depth)
                { // Z direction
                    // Calculate direct and wrap-around distances
                    int direct_dist = std::abs(current_depth - dest_depth);
                    int wrap_dist = depths - direct_dist;

                    if (current_depth < dest_depth)
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_depth++;
                        }
                        else
                        {
                            current_depth--;
                            if (current_depth < 0)
                                current_depth += depths;
                        }
                    }
                    else
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_depth--;
                        }
                        else
                        {
                            current_depth++;
                            if (current_depth >= depths)
                                current_depth -= depths;
                        }
                    }
                    next_node = current_row * cols * depths + current_col * depths + current_depth;
                }
            }
            else
            {
                if (current_row != dest_row)
                { // Y direction
                    // Calculate direct and wrap-around distances
                    int direct_dist = std::abs(current_row - dest_row);
                    int wrap_dist = rows - direct_dist;

                    if (current_row < dest_row)
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_row++;
                        }
                        else
                        {
                            current_row--;
                            if (current_row < 0)
                                current_row += rows;
                        }
                    }
                    else
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_row--;
                        }
                        else
                        {
                            current_row++;
                            if (current_row >= rows)
                                current_row -= rows;
                        }
                    }
                    if (depths > 1)
                    {
                        next_node = current_row * cols * depths + current_col * depths + current_depth;
                    }
                    else
                    {
                        next_node = current_row * cols + current_col;
                    }
                }
                else if (current_col != dest_col)
                { // X direction
                    // Calculate direct and wrap-around distances
                    int direct_dist = std::abs(current_col - dest_col); // clockwise or anti clockwise
                    int wrap_dist = cols - direct_dist;

                    if (current_col < dest_col)
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_col++;
                        }
                        else
                        {
                            current_col--;
                            if (current_col < 0)
                                current_col += cols;
                        }
                    }
                    else
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_col--;
                        }
                        else
                        {
                            current_col++;
                            if (current_col >= cols)
                                current_col -= cols;
                        }
                    }
                    if (depths > 1)
                    {
                        next_node = current_row * cols * depths + current_col * depths + current_depth;
                    }
                    else
                    {
                        next_node = current_row * cols + current_col;
                    }
                }
                else if (depths > 1 && current_depth != dest_depth)
                { // Z direction
                    // Calculate direct and wrap-around distances
                    int direct_dist = std::abs(current_depth - dest_depth);
                    int wrap_dist = depths - direct_dist;

                    if (current_depth < dest_depth)
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_depth++;
                        }
                        else
                        {
                            current_depth--;
                            if (current_depth < 0)
                                current_depth += depths;
                        }
                    }
                    else
                    {
                        if (direct_dist <= wrap_dist)
                        {
                            current_depth--;
                        }
                        else
                        {
                            current_depth++;
                            if (current_depth >= depths)
                                current_depth -= depths;
                        }
                    }
                    next_node = current_row * cols * depths + current_col * depths + current_depth;
                }
            }
        }
        else if (topology->get_topology_name() == "Mesh")
        {
            int current_row = current_node / cols;
            int current_col = current_node % cols;

            if (xy_first)
            {
                if (current_col != dest_col)
                {
                    // Move in the X direction
                    if (current_col < dest_col)
                    {
                        next_node = current_node + 1;
                    }
                    else
                    {
                        next_node = current_node - 1;
                    }
                }
                else if (current_row != dest_row)
                {
                    // Move in the Y direction
                    if (current_row < dest_row)
                    {
                        next_node = current_node + cols;
                    }
                    else
                    {
                        next_node = current_node - cols;
                    }
                }
            }
            else
            {
                if (current_row != dest_row)
                {
                    // Move in the Y direction
                    if (current_row < dest_row)
                    {
                        next_node = current_node + cols;
                    }
                    else
                    {
                        next_node = current_node - cols;
                    }
                }
                else if (current_col != dest_col)
                {
                    // Move in the X direction
                    if (current_col < dest_col)
                    {
                        next_node = current_node + 1;
                    }
                    else
                    {
                        next_node = current_node - 1;
                    }
                }
            }
        }
        Link *link = topology->get_link(current_node, next_node);
        if (!link->getActive())
        {
            std::cout << "Routing fault: Link is inactive between " << current_node << " and " << next_node << "\n";
            return std::vector<int>();
        }
        link->update_vc_load(vc, flow);
        // link->update_max_load();
        current_node = next_node;
        path.push_back(current_node);
    }
    hopcount = path.size() - 1;
    return path;
}