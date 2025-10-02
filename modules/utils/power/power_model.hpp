#include "../nodes/Node.hpp"
#include "../links/Link.hpp"
#include "../networks/Topology.hpp"
#include "../milps/noSplitFlowParser.hpp"
#include "../milps/multiSplitFlowParser.hpp"
#include "../milps/CDGFlowParser.hpp"
#include <vector>
double calculate_detailed_power_for_route(const std::vector<int> &path, double /*flow*/, Topology *topology, double link_capacitance,
                                          double link_leakage_current, double voltage, double frequency,
                                          double link_activity_factor, double buffer_read_write_factor,
                                          double crossbar_traversal_factor, double node_buffer_capacitance,
                                          double node_crossbar_capacitance, double node_leakage_current)
{
    double total_power_nodes = 0.0; // Total power for all nodes in the path
    double total_power_links = 0.0; // Total power for all links in the path
    // Calculate power for each node in the path
    for (int node_id : path)
    {
        Node *node = topology->get_node(node_id);
        if (node)
        {
            if (!node->get_status()) // If not visited before, set detailed power parameters
            {
                node->setDetailedPower(node_buffer_capacitance, node_crossbar_capacitance, voltage, frequency, node_leakage_current);
            }

            // Calculate dynamic and static power for the node
            double node_dynamic_power = node->calculate_buffer_dynamic_power(buffer_read_write_factor) +
                                        node->calculate_crossbar_dynamic_power(crossbar_traversal_factor);
            double node_static_power = node->calculate_static_power_detailed();
            double total_power_per_node = node_dynamic_power + node_static_power;

            // Mark the node as visited and update its total power
            if (!node->get_status())
            {
                node->set_status(true);
            }
            node->increment_total_power(node_dynamic_power);

            // Accumulate the total power for all nodes
            total_power_nodes += total_power_per_node;
        }
        else
        {
            throw std::runtime_error("Node not found in topology.");
        }
    }

    // Calculate power for each link in the path
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        int src_node_id = path[i];
        int dest_node_id = path[i + 1];
        Link *link = topology->get_link(src_node_id, dest_node_id);
        if (link)
        {
            if (!link->get_status()) // If not visited before, set detailed power parameters
            {
                link->set_detailed_power_parameters(link_capacitance, link_leakage_current, voltage, frequency);
            }

            // Calculate dynamic and static power for the link
            double link_dynamic_power = link->calculate_detailed_dynamic_power(link_activity_factor);
            double link_static_power = link->calculate_detailed_static_power();
            double total_power_per_link = link_dynamic_power + link_static_power;

            // Mark the link as visited and update its total power
            if (!link->get_status())
            {
                link->set_status(true);
            }
            link->increment_total_power(link_dynamic_power);

            // Accumulate the total power for all links
            total_power_links += total_power_per_link;
        }
        else
        {
            throw std::runtime_error("Link not found in topology.");
        }
    }

    // Return the total power for the route (nodes + links)
    return total_power_nodes + total_power_links;
}
double calculate_total_power(Topology *topology)
{
    double total_power = 0.0;

    // Calculate total power for all nodes
    for (int i = 0; i < topology->get_num_nodes(); ++i)
    {
        Node *node = topology->get_node(i);
        if (node)
        {
            total_power += node->get_total_power();
        }
    }

    // Calculate total power for all links
    const std::vector<Link *> &links = topology->get_all_links();
    for (Link *link : links)
    {
        if (link)
        {
            total_power += link->get_total_power();
            // link->get_max_load(); // Update the max load for the link
        }
    }

    return total_power;
}
// Function to get the maximum load across all links in the topology
double get_max_load(Topology *topology)
{
    double max_load = 0.0;
    // Calculate max load for all links
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

void calculate_power_no_splits(noSplitFlowData &data, // Use the no-splits noSplitFlowData type
                               Topology *topology,    // Ensure this is not null before calling!
                               double &total_power_accumulator, double link_capacitance,
                               double link_leakage_current, double voltage, double frequency,
                               double link_activity_factor, double buffer_read_write_factor,
                               double crossbar_traversal_factor, double node_buffer_capacitance,
                               double node_crossbar_capacitance, double node_leakage_current)
{
    if (topology == nullptr)
    {
        std::cerr << "Error: Topology pointer is null in calculate_power_no_splits." << std::endl;
        return;
    }

    // Iterate through actual flows present in data.routes
    for (const auto &flow_entry : data.routes)
    {
        int flow_id = flow_entry.first;
        const std::vector<int> &route = flow_entry.second;

        if (route.empty())
        {
            // std::cout << "Info: Route for flow " << flow_id << " is empty. Skipping." << std::endl;
            continue; // Skip power calculation for empty routes
        }

        double routeWeight = 0.0;
        auto it_flow_weight = data.flow_weights.find(flow_id);
        if (it_flow_weight != data.flow_weights.end())
        {
            routeWeight = it_flow_weight->second;
        }
        else
        {
            std::cerr << "Warning: Weight for flow " << flow_id << " not found in flow_weights. Using 0.0." << std::endl;
        }

        double power_for_this_route = 0.0;
        power_for_this_route = calculate_detailed_power_for_route(route, routeWeight, topology, link_capacitance,
                                                                  link_leakage_current, voltage, frequency,
                                                                  link_activity_factor, buffer_read_write_factor,
                                                                  crossbar_traversal_factor, node_buffer_capacitance,
                                                                  node_crossbar_capacitance, node_leakage_current);
        total_power_accumulator += power_for_this_route;
    }

    
    data.total_power = total_power_accumulator;
}
void calculate_power_multi_split_flow(multiSplitFlowData &data,
                                      Topology *topology,
                                      double &total_power_per_routes_accumulator, double link_capacitance,
                                      double link_leakage_current, double voltage, double frequency,
                                      double link_activity_factor, double buffer_read_write_factor,
                                      double crossbar_traversal_factor, double node_buffer_capacitance,
                                      double node_crossbar_capacitance, double node_leakage_current)
{
    if (topology == nullptr)
    {
        std::cerr << "Error: Topology pointer is null in calculate_power_multi_split_flow." << std::endl;
        return; // Or throw an exception
    }

    // Iterate through actual flows present in data.routes
    for (const auto &flow_entry : data.routes)
    {
        int flow_id = flow_entry.first;
        const auto &splits_for_flow = flow_entry.second;

        // Iterate through actual splits for the current flow_id
        for (const auto &split_entry : splits_for_flow)
        {
            int split_id = split_entry.first;
            const std::vector<int> &route = split_entry.second;

            if (route.empty())
            {
                // std::cout << "Info: Route for flow " << flow_id << ", split " << split_id << " is empty. Skipping." << std::endl;
                continue; // Skip power calculation for empty routes
            }

            double routeWeight = 0.0;
            auto it_flow_weights = data.flow_split_weights.find(flow_id);
            if (it_flow_weights != data.flow_split_weights.end())
            {
                const auto &splits_weights_map = it_flow_weights->second;
                auto it_split_weight = splits_weights_map.find(split_id);
                if (it_split_weight != splits_weights_map.end())
                {
                    routeWeight = it_split_weight->second;
                }
                else
                {
                    std::cerr << "Warning: Weight for flow " << flow_id << ", split " << split_id << " not found in flow_split_weights. Using 0.0." << std::endl;
                }
            }
            else
            {
                std::cerr << "Warning: Weights for flow " << flow_id << " not found in flow_split_weights. Using 0.0 for split " << split_id << "." << std::endl;
            }

            double power_for_this_route = 0.0;

            power_for_this_route = calculate_detailed_power_for_route(route, routeWeight, topology, link_capacitance,
                                                                      link_leakage_current, voltage, frequency,
                                                                      link_activity_factor, buffer_read_write_factor,
                                                                      crossbar_traversal_factor, node_buffer_capacitance,
                                                                      node_crossbar_capacitance, node_leakage_current);
            total_power_per_routes_accumulator += power_for_this_route;
        }
    }
}


void calculate_power_CDG_flow(CDGFlowData &data,
                                      Topology *topology,
                                      double &total_power_per_routes_accumulator, double link_capacitance,
                                      double link_leakage_current, double voltage, double frequency,
                                      double link_activity_factor, double buffer_read_write_factor,
                                      double crossbar_traversal_factor, double node_buffer_capacitance,
                                      double node_crossbar_capacitance, double node_leakage_current)
{ 
    if (topology == nullptr)
    {
        std::cerr << "Error: Topology pointer is null in calculate_power_multi_split_flow." << std::endl;
        return;
    }

    // Iterate through actual flows present in data.routes
    for (const auto &flow_entry : data.routes)
    {
        int flow_id = flow_entry.first;
        const auto &route = flow_entry.second;

        if (route.empty())
        {
            std::cout << "Info: Route for flow " << flow_id << " is empty. Skipping." << std::endl;
            continue;
        }

        double routeWeight = data.flowDemand[flow_id];
        double power_for_this_route = 0.0;

        power_for_this_route = calculate_detailed_power_for_route(route, routeWeight, topology, link_capacitance,
                                                                    link_leakage_current, voltage, frequency,
                                                                    link_activity_factor, buffer_read_write_factor,
                                                                    crossbar_traversal_factor, node_buffer_capacitance,
                                                                    node_crossbar_capacitance, node_leakage_current);
        total_power_per_routes_accumulator += power_for_this_route;
    }
    
}