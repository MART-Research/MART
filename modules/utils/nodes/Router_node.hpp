#ifndef ROUTER_NODE_H
#define ROUTER_NODE_H

#include "../nodes/Node.hpp" // Include the Node class
#include <vector>
#include <unordered_set>
#include <iostream>
#include <string>

class Router_node : public Node {
private:
    static int global_id; // Static variable to keep track of the number of routers
    int router_id;        // Unique ID for the Router_node
    int group_id;         // Group ID for the Router_node
    std::string router_name; // Router_node name (e.g., "R0", "R1", etc.)
    std::vector<int> connected_nodes; // Nodes connected to this Router_node
    std::unordered_set<std::string> neighbors; // Intra-group Router_node-to-Router_node connections
    std::unordered_set<std::string> global_neighbors; // Inter-group Router_node-to-Router_node connections

public:
    // Constructor
    Router_node(int id, int group)
        : Node(group), // Call the Node constructor to initialize group and other attributes
          router_id(id), group_id(group) {
        router_name = "R" + std::to_string(global_id); // Set Router_node name
        global_id++;
    }
    void set_router_name(std::string name) {
        router_name = name; // Set Router_node name
    }
    // Destructor
    ~Router_node() {
        --global_id; // Decrement global ID counter
    }

    // Add a local node to the Router_node
    void add_local_node(int id) {
        connected_nodes.push_back(id);
    }

    // Add a neighbor Router_node
    void add_neighbor(std::string id, bool is_global = false) {
        if (is_global)
            global_neighbors.insert(id);
        else
            neighbors.insert(id);
    }

    // Getters
    int get_id() const { return router_id; }
    int get_group_id() const { return group_id; }
    const std::vector<int>& get_nodes() const { return connected_nodes; }
    const std::unordered_set<std::string>& get_local_neighbors() const { return neighbors; }
    const std::unordered_set<std::string>& get_global_neighbors() const { return global_neighbors; }
    
    std::string get_router_name() const { return router_name; }

    // Print Router_node information
    void print_info() const {
        std::cout << "Router_node " << router_id << " (Group " << group_id << "):\n";
        std::cout << "  Router_node Name: " << router_name << "\n";
        std::cout << "  Connected Nodes: ";
        for (int node : connected_nodes)
            std::cout << node << " ";
        std::cout << "\n  Intra-Group Neighbors: ";
        for (const std::string& r : neighbors)
            std::cout << r << " ";
        std::cout << "\n  Global Neighbors: ";
        for (const std::string& g : global_neighbors)
            std::cout << g << " ";
        std::cout << std::endl;
    }
    // int get_group_id() const { return group_id; }
    //return my local and global neighbors IDs
    std::vector<int>get_neighbors(){
        std::vector<int> all_neighbors;
        for (const auto& neighbor : neighbors) {
            all_neighbors.push_back(std::stoi(neighbor.substr(1))); // Convert "R0" to 0
        }
        for (const auto& global_neighbor : global_neighbors) {
            all_neighbors.push_back(std::stoi(global_neighbor.substr(1))); // Convert "R0" to 0
        }
        return all_neighbors;
    }

    // Print the Router_node's connected_nodes
    void print_connected_nodes() const {
        std::cout << "Connected Nodes: ";
        for (int node : connected_nodes) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    }

};
// int Router_node::global_id = 0; // Initialize static variable
#endif // ROUTER_NODE_H