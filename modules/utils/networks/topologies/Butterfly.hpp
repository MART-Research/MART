#ifndef BUTTERFLY_H
#define BUTTERFLY_H

#include "../Topology.hpp"
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <cmath>
#include <random>

#include <stdexcept>
#include <algorithm>

class Butterfly : public Topology
{
private:
    int k; // k-ary butterfly
    int n; // #stages = log2(k) + 1
    int num_vcs;
    std::vector<PhysicalNode> map_nodes;



public:
    Butterfly(int k_param, int vcs = 1,float faulty_links=0)
        : k(k_param), num_vcs(vcs)
    {
        if ((k_param & (k_param - 1)) != 0)
        {
            throw std::invalid_argument("k must be a power of 2.");
        }
        n = static_cast<int>(std::log2(k)) + 1;
        name = "Butterfly";
        // Create nodes: k * n nodes (k switches per stage, n stages)
        for (int i = 0; i < k * n; ++i)
        {
            nodes.push_back(new Node());
        }
        create_topology();
        corrupt_links(faulty_links); // corrupt links based on the percentage
    }

    void create_topology() override
    {
        // Mapper
        map_nodes.clear();
        int total_nodes = k * n;
        map_nodes.reserve(total_nodes);
        for (int lvl = 0; lvl < n; ++lvl)
        {
            for (int idx = 0; idx < k; ++idx)
            {
                PhysicalNode pn;
                pn.id = lvl * k + idx;
                pn.x = lvl; // level
                pn.y = idx; // position within the level
                pn.z = 0;
                map_nodes.push_back(pn);
            }
        }

        // Create butterfly connections
        for (int stage = 0; stage < n - 1; ++stage)
        {
            for (int switch_num = 0; switch_num < k; ++switch_num)
            {
                int current_node = stage * k + switch_num;

                // Calculate connected nodes in next stage
                int next_stage = stage + 1;
                int bit_position = n - stage - 2;
                int mask = 1 << bit_position;
                int upper_switch = switch_num ^ mask;
                int lower_switch = switch_num;

                int upper_node = next_stage * k + upper_switch;
                int lower_node = next_stage * k + lower_switch;

                add_link(current_node, upper_node, num_vcs);
                add_link(current_node, lower_node, num_vcs);
            }
        }
    }

    std::string get_topology_name() const override
    {
        return name;
    }

    int get_num_nodes() const override
    {
        return k * n;
    }

    // implement pure_virtual functions from Topology class
    int get_num_rows() const override { return n; }
    int get_num_cols() const override { return k; }
    int get_depth() const override { return 1; } //  useless

    // Butterfly specific functions
    int get_k() const { return k; }
    int get_n() const { return n; }
    int get_num_vcs() const { return num_vcs; }

    void print_nodes_list() const
    {
        for (const auto &node : nodes)
        {
            node->print_node_info();
            std::cout << std::endl;
        }
    }

    void print_topology() const override
    {
        for (const auto &entry : links_map)
        {
            entry.second.print_info();
        }
    }

    int get_Random_Node(int src, int dest) const override
    {
        std::uniform_int_distribution<int> dist(0, get_num_nodes() - 1);
        int randnode = src;
        while (randnode == src || randnode == dest)
            randnode = dist(rng);
        return randnode;
    }

    virtual Node *get_node(int node_id) const
    {
        if (node_id >= 0 && node_id < nodes.size())
        {
            return nodes[node_id];
        }
        else
        {
            std::cerr << "Node ID out of range or does not exist: " << node_id << std::endl;
            return nullptr;
        }
    }

    // Get node by id; calculate level and index from id.
    PhysicalNode get_map_node(int id) const override
    {
        if (id < 0 || id >= get_num_nodes())
            throw std::out_of_range("Invalid node id in ButterflyTopology");
        return map_nodes[id];
    }

    // Distance metric: Manhattan distance on (level, index).
    // This is defined as |(level_a - level_b)| + |(index_a - index_b)|.
    int distance(const PhysicalNode &a, const PhysicalNode &b) const override
    {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

};

#endif // BUTTERFLY_H