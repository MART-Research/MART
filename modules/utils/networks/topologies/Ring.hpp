#ifndef RING_H
#define RING_H

#include "../Topology.hpp"
#include <iostream>
#include <vector>
#include <utility>
#include <cstdlib> 
#include <ctime>   

#include <stdexcept>
#include <cmath>
#include <algorithm>
#include "helper_function/RingHelper.hpp"

class Ring : public Topology
{
private:
    int num_nodes;
    int num_vcs;
    std::vector<PhysicalNode> map_nodes;

public:
    Ring(int n, int num_vcs = 1,float faulty_links=0) : num_nodes(n), num_vcs(num_vcs)
    {
        name = "Ring";
        //, rng(std::chrono::steady_clock::now().time_since_epoch().count())
        for (int i = 0; i < num_nodes; ++i)
        {
            nodes.push_back(new Node());
        }
        create_topology();
        corrupt_links(faulty_links); // Corrupt links based on the percentage
    }

    // create
    void create_topology() override
    {
        map_nodes.clear();
        for (int i = 0; i < num_nodes; ++i)
        {
            // Mapper
            PhysicalNode pn;
            pn.id = i;
            pn.x = i; // Position on the ring.
            pn.y = 0;
            pn.z = 0;
            map_nodes.push_back(pn);

            // Router
            int right_neighbor = (i + 1) % num_nodes;
            int left_neighbor = (i - 1 + num_nodes) % num_nodes;

            add_link(i, right_neighbor, num_vcs); // link to the right neighbor
            add_link(i, left_neighbor, num_vcs);  // link to the left neighbor
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
        return 1;
    }
    int get_num_cols() const override
    {
        return num_nodes;
    }

    Node *get_node(int node_id) const override
    {
        if (node_id >= 0 && node_id < nodes.size())
        {
            return nodes[node_id];
        }
        else
        {
            std::cerr << "Node ID out of range: " << node_id << std::endl;
            return nullptr;
        }
    }

    PhysicalNode get_map_node(int id) const override
    {
        if (id < 0 || id >= num_nodes)
        {
            throw std::out_of_range("Invalid node id in RingTopology");
        }
        return map_nodes[id];
    }

    // Print the topology's structure
    void print_topology() const override
    {
        std::cout << "Ring Topology:" << std::endl;
        for (const auto &link : links_map)
        {
            link.second.print_info();
        }
    }
    int get_Random_Node(int src, int dest) const override
    {
        std::uniform_int_distribution<> dist(0, num_nodes - 1);

        int randnode = src;
        while (randnode == src || randnode == dest)
            randnode = dist(rng); // use inherited rng
        return randnode;
    }

    int get_Random_Node_In_Quadrant(const std::vector<int> &src_coords, const std::vector<int> &dest_coords) const override
    {
        return get_Random_Node_In_Quadrant_Helper_Ring(src_coords, dest_coords, num_nodes, rng);
    }

    // Distance on a ring: min(|a - b|, num_nodes - |a - b|)
    int distance(const PhysicalNode &a, const PhysicalNode &b) const override
    {
        int diff = std::abs(a.x - b.x);
        return std::min(diff, num_nodes - diff);
    }
};

#endif
