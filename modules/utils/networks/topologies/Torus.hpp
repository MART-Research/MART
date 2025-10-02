#ifndef TORUS_H
#define TORUS_H

#include "../Topology.hpp"
#include "helper_function/TorusHelper.hpp"
#include <iostream>
#include <vector>
#include <utility>
#include <cstdlib>
#include <ctime>
#include <stdexcept>
#include <cmath>

class Torus : public Topology
{
private:
    int rows, cols, depth;
    int num_vcs;
    std::vector<PhysicalNode> map_nodes;

public:
    // Constructor: depth=1 for 2D, depth>1 for 3D
    Torus(int r, int c, int d = 1, int num_vcs = 1, float faulty_links = 0)
        : rows(r), cols(c), depth(d), num_vcs(num_vcs)
    {
        if (depth < 1)
        {
            throw std::invalid_argument("Torus requires depth >= 1");
        }
        name = (depth == 1) ? "Torus_2D" : "Torus_3D";
        srand(time(0));
        int total_nodes = rows * cols * depth;
        for (int i = 0; i < total_nodes; ++i)
        {
            nodes.push_back(new Node());
        }
        create_topology();
        corrupt_links(faulty_links);
    }

    void create_topology() override
    {
        int total_nodes = rows * cols * depth;
        map_nodes.clear();
        map_nodes.reserve(total_nodes);
        for (int i = 0; i < total_nodes; ++i)
        {
            int x = i % cols;
            int y = (i / cols) % rows;
            int z = i / (rows * cols);
            map_nodes.push_back({i, x, y, z});
        }
        if (depth == 1)
        {
            // 2D Torus
            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    int node = i * cols + j;
                    add_link(node, ((i - 1 + rows) % rows) * cols + j, num_vcs); // up
                    add_link(node, ((i + 1) % rows) * cols + j, num_vcs);        // down
                    add_link(node, i * cols + (j + 1) % cols, num_vcs);          // right
                    add_link(node, i * cols + (j - 1 + cols) % cols, num_vcs);   // left
                }
            }
        }
        else
        {
            // 3D Torus
            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    for (int k = 0; k < depth; ++k)
                    {
                        int node = i * cols * depth + j * depth + k;
                        add_link(node, i * cols * depth + ((j + 1) % cols) * depth + k, num_vcs);        // right
                        add_link(node, i * cols * depth + ((j - 1 + cols) % cols) * depth + k, num_vcs); // left
                        add_link(node, ((i + 1) % rows) * cols * depth + j * depth + k, num_vcs);        // down
                        add_link(node, ((i - 1 + rows) % rows) * cols * depth + j * depth + k, num_vcs); // up
                        add_link(node, i * cols * depth + j * depth + (k + 1) % depth, num_vcs);         // front
                        add_link(node, i * cols * depth + j * depth + (k - 1 + depth) % depth, num_vcs); // back
                    }
                }
            }
        }
    }

    int get_num_nodes() const override
    {
        return rows * cols * depth;
    }
    int get_num_rows() const override { return rows; }
    int get_num_cols() const override { return cols; }
    int get_depth() const override { return depth; }
    std::string get_topology_name() const override { return name; }

    // Print the topology's structure
    void print_topology() const override
    {
        std::cout << "Topology:" << std::endl;
        for (const auto &entry : links_map)
        {
            entry.second.print_info();
        }
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
        if (id < 0 || id >= get_num_nodes())
            throw std::out_of_range("Invalid node id in Torus");
        return map_nodes[id];
    }

    int get_Random_Node(int src, int dest) const override
    {
        std::uniform_int_distribution<> dist(0, rows * cols * depth - 1);
        int randnode = src;
        while (randnode == src || randnode == dest)
            randnode = dist(rng);
        return randnode;
    }
    int get_Random_Node_In_Quadrant(const std::vector<int> &src_coords, const std::vector<int> &dest_coords) const override
    {
        return get_Random_Node_In_Quadrant_Helper(src_coords, dest_coords, cols, depth, rng);
    }
    int distance(const PhysicalNode &a, const PhysicalNode &b) const override
    {
        if (depth == 1)
        {
            int dx = std::abs(a.x - b.x);
            int dy = std::abs(a.y - b.y);
            dx = std::min(dx, cols - dx);
            dy = std::min(dy, rows - dy);
            return dx + dy;
        }
        else
        {
            int dx = std::abs(a.x - b.x);
            int dy = std::abs(a.y - b.y);
            int dz = std::abs(a.z - b.z);
            dx = std::min(dx, cols - dx);
            dy = std::min(dy, rows - dy);
            dz = std::min(dz, depth - dz);
            return dx + dy + dz;
        }
    }
};

#endif
