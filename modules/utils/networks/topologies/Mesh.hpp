#ifndef MESH_H
#define MESH_H

#include "../Topology.hpp"
#include "helper_function/MeshHelper.hpp"
#include <iostream>
#include <vector>
#include <utility> // for std::pair
#include <cstdlib> // Include for rand() and srand()
#include <ctime>   // Include for time()
#include <string>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>
#include <cmath>

class Mesh : public Topology
{
private:
    int rows, cols;
    int num_vcs;

    std::vector<PhysicalNode> map_nodes;

    // cycle handling functions
    //  Helper function for DFS to detect cycles
    bool dfs(int node, std::unordered_set<int> &visited, std::unordered_set<int> &recursion_stack)
    {
        if (recursion_stack.find(node) != recursion_stack.end())
        {
            // Node is already in the current recursion stack, indicating a cycle
            return true;
        }
        if (visited.find(node) != visited.end())
        {
            // Already visited this node in a previous DFS path, no cycle here
            return false;
        }

        // Mark the node as visited and part of the current recursion stack
        visited.insert(node);
        recursion_stack.insert(node);

        // Get all neighbors (i.e., connected nodes via links)
        for (const auto &link_entry : links_map)
        {
            const Link &link = link_entry.second;
            int neighbor = (link.get_node1() == node) ? link.get_node2() : link.get_node1();
            if (dfs(neighbor, visited, recursion_stack))
            {
                return true; // Cycle found
            }
        }

        // Backtrack: Remove the node from recursion stack after processing
        recursion_stack.erase(node);
        return false;
    }

    // Function to remove an edge (link) from the graph
    void remove_edge(int node1, int node2)
    {
        std::string key = generate_key(node1, node2);
        if (links_map.find(key) != links_map.end())
        {
            links_map.erase(key); // Remove the link between node1 and node2
        }
    }

    // Function to remove cycles from the mesh (graph)
    void remove_cycles()
    {
        std::unordered_set<int> visited;
        std::unordered_set<int> recursion_stack;

        // We need to check for cycles from every node
        for (int node = 0; node < rows * cols; ++node)
        {
            if (visited.find(node) == visited.end())
            {
                // Start DFS from each unvisited node
                if (dfs(node, visited, recursion_stack))
                {
                    // If a cycle is detected, remove the last added edge that caused the cycle
                    // For simplicity, let's assume removing the first found edge causing the cycle
                    for (const auto &link_entry : links_map)
                    {
                        const Link &link = link_entry.second;
                        int neighbor = (link.get_node1() == node) ? link.get_node2() : link.get_node1();
                        if (recursion_stack.find(neighbor) != recursion_stack.end())
                        {
                            remove_edge(node, neighbor);
                            break; // Remove only one edge causing the cycle
                        }
                    }
                }
            }
        }
    }

public:
    // Constructor to initialize mesh dimensions and number of VCs
    Mesh(int r, int c, int vcs = 1,float faulty_links=0) : rows(r), cols(c), num_vcs(vcs)
    {
        name = "Mesh";
        // rng(std::chrono::steady_clock::now().time_since_epoch().count())
        //  srand(time(0)); // Seed the random number generator
        for (int i = 0; i < rows * cols; ++i)
        {
            nodes.push_back(new Node());
        }
        create_topology();
        corrupt_links(faulty_links); // Corrupt links based on the percentage
    }

    // Create the mesh topology by adding links between nodes
    void create_topology() override
    {
        // Mapper
        int total_nodes = rows * cols;
        map_nodes.clear();
        map_nodes.reserve(total_nodes);
        for (int i = 0; i < total_nodes; ++i)
        {
            int x = i % cols;
            int y = i / cols;
            PhysicalNode pn = {i, x, y, 0};
            map_nodes.push_back(pn);
        }

        // Router
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                int node = i * cols + j;
                if (j < cols - 1)
                {
                    add_link(node, node + 1, num_vcs); // Horizontal link
                    nodes[node]->add_neighbor(node + 1);
                    nodes[node + 1]->add_neighbor(node);
                }
                if (i < rows - 1)
                {

                    add_link(node, node + cols, num_vcs); // Vertical link
                    nodes[node]->add_neighbor(node + cols);
                    nodes[node + cols]->add_neighbor(node);
                }
            }
        }
    }

    std::string get_topology_name() const override
    {
        return name;
    }

    // Return the total number of nodes
    int get_num_nodes() const override
    {
        return rows * cols;
    }

    // Return the number of rows
    int get_num_rows() const override
    {
        return rows;
    }

    // Return the number of columns
    int get_num_cols() const override
    {
        return cols;
    }

    // Return the links map
    const std::unordered_map<std::string, Link> &get_links_map() const
    {
        return links_map;
    }

    // Print the topology's structure
    void print_topology() const override
    {
        std::cout << "Topology:" << std::endl;
        for (const auto &entry : links_map)
        {
            entry.second.print_info();
        }
    }

    int get_Random_Node(int src, int dest) const override
    {
        std::uniform_int_distribution<int> dist(0, rows * cols - 1);
        int randnode = src;
        while (randnode == src || randnode == dest)
            randnode = dist(rng);
        return randnode;
    }
    int get_Random_Node_In_Quadrant(const std::vector<int> &src_coords, const std::vector<int> &dest_coords) const override
    {
        return get_Random_Node_In_Quadrant_Helper_Mesh(src_coords, dest_coords, cols, rng);
    }

    // Public method to trigger cycle removal
    void convert_to_acyclic()
    {
        remove_cycles(); // Calls the internal function to remove cycles
    }

    // Function to get a node by its ID
    Node *get_node(int node_id) const override
    {
        if (node_id >= 0 && node_id < nodes.size())
        {
            return nodes[node_id];
        }
        else
        {
            std::cerr << "Node ID out of range: " << node_id << std::endl;
            return nullptr; // Return null if the node ID is invalid
        }
    }

    PhysicalNode get_map_node(int id) const override
    {
        if (id < 0 || id >= get_num_nodes())
            throw std::out_of_range("Invalid node id in Mesh2D");
        return map_nodes[id];
    }

    void print_nodes_list() const
    {
        for (const auto &node : nodes)
        {
            node->print_node_info();
            std::cout << std::endl;
        }
    }

    // For a 2D mesh, we use Manhattan distance.
    int distance(const PhysicalNode &a, const PhysicalNode &b) const override
    {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }
};

#endif // MESH_H
