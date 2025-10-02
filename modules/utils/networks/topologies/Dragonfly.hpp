#ifndef DRAGONFLY_H
#define DRAGONFLY_H

#include "../Topology.hpp"
#include "../../nodes/Router_node.hpp"
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <random>
class Dragonfly : public Topology
{
private:
    int num_groups;        // Number of groups
    int routers_per_group; // Number of routers per group
    int nodes_per_router;  // Number of nodes per router
    int global_links;      // Number of global links per router
    int num_vcs;
    std::vector<Router_node *> routers; // Changed to Router* to match the Dragonfly topology
    std::vector<PhysicalNode> map_nodes;

public:
    Dragonfly(int groups, int routers, int nodes_per_rtr, int globals, int vcs = 1, float faulty_links = 0.0)
        : num_groups(groups), routers_per_group(routers), nodes_per_router(nodes_per_rtr),
          global_links(globals), num_vcs(vcs)
    {
        name = "Dragonfly";
        for (int i = 0; i < num_groups * routers_per_group; ++i)
        {
            nodes.push_back(new Node());
        }
        create_topology();
        corrupt_links(faulty_links); // Corrupt links based on the given percentage
    }

    void create_topology() override
    {
        int total_routers = num_groups * routers_per_group;
        int total_nodes = total_routers * nodes_per_router;
        // mapper related nodes
        int nodes_per_group = total_nodes / num_groups;
        map_nodes.clear();
        for (int i = 0; i < total_nodes; i++)
        {
            PhysicalNode pn = {i, i / nodes_per_group, i % nodes_per_group, 0};
            map_nodes.push_back(pn);
        }

        for (int i = 0; i < total_nodes; ++i)
        {
            nodes.push_back(new Node());
        }

        routers.resize(total_routers, nullptr);

        // Create routers and place them in both `routers` and `nodes`
        for (int g = 0; g < num_groups; ++g)
        {
            for (int r = 0; r < routers_per_group; ++r)
            {
                int router_id = g * routers_per_group + r;
                Router_node *router = new Router_node(router_id, g);      // Create a Router object
                router->set_router_name("R" + std::to_string(router_id)); // Assign a name to the router
                router->set_router_id(router_id);                         // Set the router_id to its unique ID
                routers[router_id] = router;
                nodes[router_id] = router;
            }
        }

        // Intra-group router-to-router connections (fully connected within each group)
        for (int g = 0; g < num_groups; ++g)
        {
            for (int r1 = 0; r1 < routers_per_group; ++r1)
            {
                int router1 = g * routers_per_group + r1;
                for (int r2 = r1 + 1; r2 < routers_per_group; ++r2)
                {
                    int router2 = g * routers_per_group + r2;

                    add_link(router1, router2, num_vcs);

                    Router_node *rtr1 = dynamic_cast<Router_node *>(nodes[router1]);
                    Router_node *rtr2 = dynamic_cast<Router_node *>(nodes[router2]);
                    if (rtr1 && rtr2)
                    {
                        rtr1->add_neighbor("R" + std::to_string(rtr2->get_id()), false);
                        rtr2->add_neighbor("R" + std::to_string(rtr1->get_id()), false);
                    }
                }
            }
        }

        // Deterministic global inter-group router-to-router links
        for (int g = 0; g < num_groups; ++g)
        {
            for (int r = 0; r < routers_per_group; ++r)
            {
                int src_router = g * routers_per_group + r;

                for (int gl = 0; gl < global_links; ++gl)
                {
                    int dest_group = (g + gl + 1) % num_groups; // simple round-robin
                    int dest_router_index = (r + gl) % routers_per_group;
                    int dest_router = dest_group * routers_per_group + dest_router_index;

                    // Prevent duplicate links (check if already exists)
                    std::string key = generate_key(src_router, dest_router);
                    if (links_map.find(key) != links_map.end())
                        continue;

                    add_link(src_router, dest_router, num_vcs);

                    Router_node *src = dynamic_cast<Router_node *>(nodes[src_router]);
                    Router_node *dest = dynamic_cast<Router_node *>(nodes[dest_router]);

                    if (src && dest)
                    {
                        src->add_neighbor("R" + std::to_string(dest->get_id()), true);
                        dest->add_neighbor("R" + std::to_string(src->get_id()), true);
                    }
                }
            }
        }

        // Node-to-router connections
        for (int r = 0; r < total_routers; ++r)
        {
            for (int n = 0; n < nodes_per_router; ++n)
            {
                int node_id = total_routers + r * nodes_per_router + n;

                add_link(node_id, r, num_vcs);
                // Set group_id and router_id for the node
                nodes[node_id]->set_group(r / routers_per_group); // Group ID is derived from the router's group
                nodes[node_id]->set_router_id(r);                 // Router ID is the ID of the router
            }
        }
    }

    // get routers
    std::vector<Router_node *> get_routers() const
    {
        return routers;
    }
    // get the indices in the routers vector that are routers
    std::vector<int> get_routers_indices() const
    {
        std::vector<int> router_indices;
        for (int i = 0; i < routers.size(); ++i)
        {
            if (dynamic_cast<Router_node *>(nodes[i]) != nullptr)
            {
                router_indices.push_back(i);
            }
        }
        return router_indices;
    }

    std::string get_topology_name() const override
    {
        return name;
    }

    int get_num_nodes() const override
    {
        return num_groups * routers_per_group * nodes_per_router;
    }

    // useless functions for Dragonfly
    int get_num_rows() const override { return num_groups; }
    int get_num_cols() const override { return routers_per_group; }
    int get_depth() const override { return 3; }
    //
    int get_nodes_per_router() const override
    {
        return nodes_per_router;
    }
    int get_num_groups() const override
    {
        return num_groups;
    }
    int get_routers_per_group() const override
    {
        return routers_per_group;
    }
    int get_global_links_per_router() const override
    {
        return global_links;
    }
    int get_num_vcs() const
    {
        return num_vcs;
    }

    const std::unordered_map<std::string, Link> &get_links_map() const
    {
        return links_map;
    }

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
        std::cout << "Dragonfly Topology:\n";
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

    Node *get_node(int node_id) const
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
    PhysicalNode get_map_node(int id) const override
    {
        int num_nodes = get_num_nodes();
        if (id < 0 || id >= num_nodes)
            throw std::out_of_range("Invalid node id in Dragonfly Topology");
        return map_nodes[id];
    }

    std::vector<int> global_routers, local_routers;
    void get_global_and_local_routers()
    {
        for (int g = 0; g < num_groups; ++g)
        {
            for (int r = 0; r < routers_per_group; ++r)
            {
                int router_id = g * routers_per_group + r;

                // Classify routers as global or local
                if (r < global_links)
                {
                    global_routers.push_back(router_id); // Global router
                }
                else
                {
                    local_routers.push_back(router_id); // Local router
                }
            }
        }
    }

    // print the neighbors of each node
    void print_neighbors() const
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            std::cout << "Node " << i << " neighbors: ";
            for (const auto &neighbor : nodes[i]->get_neighbors())
            {
                std::cout << neighbor << " ";
            }
            std::cout << std::endl;
        }
    }
    // print neighbors of routers
    void print_router_neighbors() const
    {
        for (int i = 0; i < routers.size(); ++i)
        {
            std::cout << "Router " << i << " neighbors: ";
            for (const auto &neighbor : routers[i]->get_neighbors())
            {
                std::cout << neighbor << " ";
            }
            std::cout << std::endl;
        }
    }

    void print_router_connected_nodes() const
    {
        for (int i = 0; i < routers.size(); ++i)
        {
            std::cout << "Router " << i << " connected nodes: ";
            for (const auto &node : routers[i]->get_nodes())
            {
                std::cout << node << " ";
            }
            std::cout << std::endl;
        }
    }
    void print_node_info_id() const
    {
        for (int i = 0; i < nodes.size(); ++i)
        {
            std::cout << "Node " << i << " ID: " << nodes[i]->get_id() << " group_id: " << nodes[i]->get_group() << " router_id " << nodes[i]->get_router_id() << std::endl;
        }
    }
    // Modified distance:
    // If nodes are in the same group: simply the absolute difference in their index.
    // If in different groups: use the index difference plus 5 times the absolute group difference.
    int distance(const PhysicalNode &a, const PhysicalNode &b) const override
    {
        int group_diff = std::abs(a.x - b.x);
        int index_diff = std::abs(a.y - b.y);
        if (group_diff == 0)
            return index_diff;
        return index_diff + 10 * group_diff;
    }
    ~Dragonfly()
    {
        auto routers_vec = get_routers();
        for (auto node : get_nodes_list())
        {
            if (std::find(routers_vec.begin(), routers_vec.end(), node) == routers_vec.end())
            {
                delete node;
            }
        }

        for (auto router : routers_vec)
        {
            delete router;
        }
    }
};

#endif // DRAGONFLY_H