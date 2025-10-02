#ifndef NODE_HPP
#define NODE_HPP

#include <iostream>
#include <unordered_map>
#include <vector>

class Node
{
private:
    static int next_id;                         // Static counter for unique IDs
    int node_id;                                // Unique identifier for the node
    std::unordered_map<int, int> routing_table; // Maps destination node -> next hop
    std::vector<int> neighbors;                 // List of neighboring node IDs
    int group;                                  // Group ID (optional, for group-based topologies like Dragonfly)
    int router_id;
    bool IsVisited;
    // Parameters for the detailed power model
    double buffer_capacitance;   // Capacitance of the node buffers
    double crossbar_capacitance; // Capacitance of the crossbar
    double voltage;              // Voltage (Volts)
    double frequency;            // Frequency (Hz)
    double leakage_current;      // Leakage current (Amperes)

    double total_power; // Total power (W) = dynamic power from each route passing the node + static power

public:
    // Constructor
    Node(int grp = -1, bool status = false) : node_id(next_id++), group(grp), IsVisited(status)
    {
        // Initialize detailed power model parameters with default values
        buffer_capacitance = 1e-12;   // 1 pF
        crossbar_capacitance = 2e-12; // 2 pF
        voltage = 1.0;                // 1 V
        frequency = 1e9;              // 1 GHz
        leakage_current = 2e-6;       // 2 µA
        total_power = 0;
    }

    // Getter for node ID
    int get_id() const
    {
        return node_id;
    }
    void set_status(bool status)
    {
        IsVisited = status;
    }
    bool get_status()
    {
        return IsVisited;
    }
    // Setter for the group ID (used in group-based topologies)
    void set_group(int grp)
    {
        group = grp;
    }

    void set_router_id(int id)
    {
        router_id = id;
    }
    int get_router_id() const
    {
        return router_id;
    }
    // Getter for the group ID
    int get_group() const
    {
        return group;
    }

    // Add a neighbor
    void add_neighbor(int neighbor_id)
    {
        neighbors.push_back(neighbor_id);
    }

    // Get neighbors
    std::vector<int> get_neighbors() const
    {
        return neighbors;
    }
    void reset_node()
    {
        IsVisited = false;
        total_power = 0;
    }
    // Update the routing table with a new entry
    void update_routing_table(int destination, int next_hop)
    {
        routing_table[destination] = next_hop;
    }

    // Get the next hop for a given destination
    int get_next_hop(int destination) const
    {
        auto it = routing_table.find(destination);
        if (it != routing_table.end())
        {
            return it->second;
        }
        else
        {
            std::cerr << "No route found for destination " << destination << std::endl;
            return -1; // Indicates no valid route found
        }
    }

    // Display the routing table (for debugging)
    void print_routing_table() const
    {
        std::cout << "Routing Table for Node " << node_id << ":\n";
        for (const auto &entry : routing_table)
        {
            std::cout << "Destination: " << entry.first
                      << ", Next Hop: " << entry.second << std::endl;
        }
    }

    void print_node_info() const
    {
        std::cout << "Node ID: " << node_id << ", Group ID: " << group << std::endl;
        // Print neighbors
        std::cout << "Neighbors: ";
        for (int neighbor : neighbors)
        {
            std::cout << neighbor << " ";
        }
    }
    // Power modeling methods

    // === Detailed Power Model ===
    void setDetailedPower(double buffer_cap, double crossbar_cap, double volt, double freq, double leakage)
    {
        buffer_capacitance = buffer_cap;
        crossbar_capacitance = crossbar_cap;
        voltage = volt;
        frequency = freq;
        leakage_current = leakage;
        total_power = leakage_current * voltage;
    }
    std::vector<double> getDetailedPower() const
    {
        return {buffer_capacitance, crossbar_capacitance, voltage, frequency, leakage_current};
    }
    // Calculate dynamic power for buffers
    double calculate_buffer_dynamic_power(double activity_factor) const
    {
        return activity_factor * buffer_capacitance * voltage * voltage * frequency;
    }

    // Calculate dynamic power for crossbars
    double calculate_crossbar_dynamic_power(double activity_factor) const
    {
        return activity_factor * crossbar_capacitance * voltage * voltage * frequency;
    }

    // Calculate static power (detailed model)
    double calculate_static_power_detailed() const
    {
        return leakage_current * voltage;
    }

    // Calculate total power (detailed model)
    double calculate_total_power_detailed(double buffer_activity, double crossbar_activity) const
    {
        double buffer_power = calculate_buffer_dynamic_power(buffer_activity);
        double crossbar_power = calculate_crossbar_dynamic_power(crossbar_activity);
        double static_power = calculate_static_power_detailed();
        return buffer_power + crossbar_power + static_power;
    }
    // Total power
    double get_total_power() const
    {
        return total_power;
    }
    void increment_total_power(double power)
    {
        total_power += power;
    }
    virtual ~Node() {} // Now Base is polymorphic
};
// int Node::next_id = 0;
#endif // NODE_HPP