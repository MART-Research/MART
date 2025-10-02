#ifndef TREE_H
#define TREE_H

#include "../Topology.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>

#include <stdexcept>

class Tree : public Topology
{
private:
    int num_levels;       // #levels, total num nodes=2^levels-1 (assuming a perfect tree)
    int branching_factor; // #children
    int num_vcs;
    std::unordered_map<int, std::vector<int>> adj_list;       // Adjacency list
    std::unordered_map<int, int> parent_map;                  // store parent of each node
    std::unordered_map<int, std::vector<int>> node_relations; // stores parent and children
    std::unordered_map<int, std::vector<int>> children_list;  // store children
    std::vector<PhysicalNode> map_nodes;

    void add_link(int from, int to, int vcs) override
    {
        std::string key = generate_key(from, to);
        double latency = 0.1;    // Example latency
        double bandwidth = 10.0; // Example bandwidth per VC

        if (links_map.find(key) == links_map.end())
        {
            links_map[key] = Link(from, to, latency, bandwidth, false, vcs);
            nodes[from]->add_neighbor(to);
            nodes[to]->add_neighbor(from);
        }
        adj_list[from].push_back(to);
        adj_list[to].push_back(from);
    }

    void dfs(int node, std::unordered_set<int> &visited, std::vector<int> &traversal) const
    {
        visited.insert(node);
        traversal.push_back(node);
        for (int neighbor : adj_list.at(node))
        {
            if (visited.find(neighbor) == visited.end())
            {
                dfs(neighbor, visited, traversal);
            }
        }
    }

    int parent_position(int pos) const
    {
        return pos / branching_factor;
    }

    int compute_lca_level(int level_a, int pos_a, int level_b, int pos_b) const
    {
        // Bring both nodes to the same level.
        while (level_a > level_b)
        {
            level_a--;
            pos_a = parent_position(pos_a);
        }
        while (level_b > level_a)
        {
            level_b--;
            pos_b = parent_position(pos_b);
        }
        // Now, move both upward until they match.
        while (level_a > 0 && pos_a != pos_b)
        {
            level_a--;
            level_b--;
            pos_a = parent_position(pos_a);
            pos_b = parent_position(pos_b);
        }
        return level_a;
    }

public:
    Tree(int levels, int branches, int vcs = 1,float faulty_links=0)
        : num_levels(levels), branching_factor(branches), num_vcs(vcs)
    {
        name = "Tree";
        create_topology();
        corrupt_links(faulty_links); // Corrupt links based on the percentage
    }
    ~Tree()
    {
        for (auto node : nodes)
        {
            delete node;
        }
    }
    void create_topology() override
    {
        // Mapper part
        map_nodes.clear();
        int id = 0;
        // For each level, number of nodes is branching_factor^(level)
        for (int level = 0; level < num_levels; ++level)
        {
            int level_count = static_cast<int>(std::pow(branching_factor, level));
            for (int pos = 0; pos < level_count; ++pos)
            {
                PhysicalNode pn;
                pn.id = id;
                pn.x = level; // level of the node
                pn.y = pos;   // position within the level
                pn.z = 0;
                map_nodes.push_back(pn);
                id++;
            }
        }

        // Create all nodes first
        int total_nodes = get_num_nodes();
        for (int i = 0; i < total_nodes; ++i)
        {
            nodes.push_back(new Node(i));
        }

        // Build tree connections
        for (int i = 1; i < total_nodes; ++i)
        {
            int parent = (i - 1) / branching_factor;
            parent_map[i] = parent;
            node_relations[i].push_back(parent);
            node_relations[parent].push_back(i);
            children_list[parent].push_back(i);
            add_link(parent, i, num_vcs);
        }
    }

    int get_num_nodes() const override
    {
        return (pow(branching_factor, num_levels) - 1) / (branching_factor - 1);
    }

    const std::vector<int> &get_node_relations(int node) const
    {
        return node_relations.at(node);
    }

    const std::vector<int> &get_children(int node) const
    {
        return children_list.at(node);
    }

    const std::unordered_map<int, int> &get_parent_map() const
    {
        return parent_map;
    }

    std::string get_topology_name() const override
    {
        return name;
    }

    int get_num_rows() const override { return num_levels; }
    int get_num_cols() const override { return branching_factor; }
    int get_depth() const override { return num_levels; }



    std::vector<int> get_dfs_traversal(int start_node) const
    {
        std::unordered_set<int> visited;
        std::vector<int> traversal;
        dfs(start_node, visited, traversal);
        return traversal;
    }

    void print_topology() const override
    {
        std::cout << "Tree Topology (Levels: " << num_levels
                  << ", Branching Factor: " << branching_factor << "):\n";
        for (const auto &pair : adj_list)
        {
            std::cout << "Node " << pair.first << " -> ";
            for (int neighbor : pair.second)
            {
                std::cout << neighbor << " ";
            }
            std::cout << std::endl;
        }
    }
    int get_Random_Node(int src, int /*dest*/) const override
    {
        if (adj_list.find(src) != adj_list.end() && !adj_list.at(src).empty())
        {
            std::uniform_int_distribution<int> dist(0, adj_list.at(src).size() - 1);
            int random_index = dist(rng); // Generate a random index using rng
            return adj_list.at(src)[random_index];
        }
        return src; // If no neighbors exist, return the source node
    }

    Node *get_node(int node_id) const
    {
        if (node_id >= 0 && node_id < nodes.size())
        {
            return nodes[node_id];
        }
        else
        {
            std::cerr << "Node ID out of range or does not exist: " << node_id << std::endl;
            return nullptr; // Return null if the node ID is invalid
        }
    }

    PhysicalNode get_map_node(int id) const override
    {
        if (id < 0 || id >= get_num_nodes())
            throw std::out_of_range("Invalid node id in TreeTopology");
        return map_nodes[id];
    }

    // Updated distance function using the least common ancestor (LCA).
    // Given two nodes a and b, let LCA be at level L_lca.
    // Then distance = (a.level - L_lca) + (b.level - L_lca)
    int distance(const PhysicalNode &a, const PhysicalNode &b) const override
    {
        int level_a = a.x;
        int level_b = b.x;
        int pos_a = a.y;
        int pos_b = b.y;
        int lca_level = compute_lca_level(level_a, pos_a, level_b, pos_b);
        return (a.x - lca_level) + (b.x - lca_level);
    }
};

#endif // TREE_H
