#ifndef HYPERCUBE_H
#define HYPERCUBE_H

#include "../Topology.hpp"
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <algorithm>
#include <string>
#include <random>

#include <stdexcept>

class Hypercube : public Topology
{
private:
    int dimensions;
    int num_nodes;
    int num_vcs;
    std::vector<PhysicalNode> map_nodes;


    void remove_cycles()
    {
        std::unordered_set<std::string> keep_links;
        std::vector<bool> visited(num_nodes, false);
        std::queue<int> q;

        // Start from node 0
        q.push(0);
        visited[0] = true;

        while (!q.empty())
        {
            int node = q.front();
            q.pop();

            for (int d = 0; d < dimensions; ++d)
            {
                int neighbor = node ^ (1 << d); // Flip d-th bit to get the neighbor
                std::string key = generate_key(node, neighbor);

                if (!visited[neighbor])
                {
                    visited[neighbor] = true;
                    keep_links.insert(key);
                    q.push(neighbor);
                }
            }
        }

        // Remove links that are not in the spanning tree
        for (auto it = links_map.begin(); it != links_map.end();)
        {
            if (keep_links.find(it->first) == keep_links.end())
            {
                it = links_map.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    // Compute Hamming distance between two integers.
    int hamming_distance(int a, int b) const
    {
        int x = a ^ b;
        int count = 0;
        while (x)
        {
            count += (x & 1);
            x >>= 1;
        }
        return count;
    }

public:
    // Constructor to initialize hypercube dimensions and number of VCs
    Hypercube(int dims, int vcs = 1,float faulty_links=0) : dimensions(dims), num_vcs(vcs)
    {
        name = "Hypercube";
        // , name("Hypercube")
        // , rng(std::chrono::steady_clock::now().time_since_epoch().count())
        num_nodes = 1 << dimensions; // 2^dimensions
        // reserve nodes
        nodes.reserve(num_nodes);
        for (int i = 0; i < num_nodes; ++i)
        {
            nodes.push_back(new Node(i));
        }
        create_topology();
        corrupt_links(faulty_links); // corrupt links if needed
    }

    void create_topology() override
    {
        map_nodes.clear();
        for (int node = 0; node < num_nodes; ++node)
        {
            // Mapper
            PhysicalNode pn;
            pn.id = node;
            // Store the id in the x coordinate.
            pn.x = node;
            pn.y = 0;
            pn.z = 0;
            map_nodes.push_back(pn);

            // Router
            for (int d = 0; d < dimensions; ++d)
            {
                int neighbor = node ^ (1 << d); // the d-th  bit is flipped to find the neighbor
                add_link(node, neighbor, num_vcs);
            }
        }
    }

    std::string get_topology_name() const override
    {
        return name;
    }

    int get_num_nodes() const override
    {
        return num_nodes;
    }

    int get_num_rows() const override
    {
        return 0;
    }

    int get_num_cols() const override
    {
        return 0;
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
        if (id < 0 || id >= num_nodes)
            throw std::out_of_range("Invalid node id in HypercubeTopology");
        return map_nodes[id];
    }

    //  depth=dimension
    int get_depth() const override
    {
        return dimensions;
    }

    // Return the links map
    const std::unordered_map<std::string, Link> &get_links_map() const
    {
        return links_map;
    }

    void print_topology() const override
    {
        std::cout << "Hypercube Topology (" << dimensions << " dimensions, " << num_nodes << " nodes):\n";
        for (const auto &entry : links_map)
        {
            entry.second.print_info();
        }
    }

    int get_Random_Node(int src, int dest) const override
    {
        std::uniform_int_distribution<int> dist(0, num_nodes - 1);
        int randnode = src;
        while (randnode == src || randnode == dest)
            randnode = dist(rng);
        return randnode;
    }

    // Distance is the Hamming distance of the x coordinates.
    int distance(const PhysicalNode &a, const PhysicalNode &b) const override
    {
        return hamming_distance(a.x, b.x);
    }
};

#endif // HYPERCUBE_H
