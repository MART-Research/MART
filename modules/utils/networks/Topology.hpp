#ifndef ABSTRACT_TOPOLOGY_H
#define ABSTRACT_TOPOLOGY_H

#include <vector>
#include <string>
#include <chrono>
#include <random>
#include <algorithm> // For std::sort
#include "../links/Link.hpp"
#include "../nodes/Node.hpp"
#include "../nodes/map_nodes.hpp"
#include <fstream>

enum class TopologyType
{
    Hypercube,
    Dragonfly,
    Butterfly,
    Unknown
};

// Base class for all topologies
class Topology
{
protected:
    std::string name;
    std::vector<Node *> nodes; // List of nodes in the topology
    // std::vector<Link *> links; // List of links in the topology
    mutable std::unordered_map<std::string, Link> links_map;
    mutable std::mt19937 rng;

public:
    Topology() : rng(std::chrono::steady_clock::now().time_since_epoch().count()) {}
    virtual ~Topology() {}

    // Pure virtual method to create the topology (must be implemented by derived classes)
    virtual void create_topology() = 0;

    // Pure virtual method to return the total number of nodes
    virtual int get_num_nodes() const = 0;
    virtual std::string get_topology_name() const = 0;
    // Pure virtual methods to get rows and columns (for 2D topologies like Mesh)
    virtual int get_num_rows() const
    {
        return 1;
    }
    virtual int get_num_cols() const
    {
        return 1;
    }
    virtual int get_depth() const
    {
        return 1; // means it is 2D
    } //  it is updated for 3D topologies like Torus_3D but will return 1 for 2D

    // Utility to add a link between nodes
    std::string generate_key(int node1, int node2) const
    {
        return node1 < node2 ? std::to_string(node1) + "-" + std::to_string(node2)
                             : std::to_string(node2) + "-" + std::to_string(node1);
    }
    // virtual void add_link(int from, int to, int num_vcs) = 0;
    // this would be overriden in Tree class
    virtual void add_link(int node1, int node2, int num_vcs)
    {
        std::string key = generate_key(node1, node2);
        double latency = 0.1;
        double bandwidth = 10.0;

        if (links_map.find(key) == links_map.end())
        {
            links_map[key] = Link(node1, node2, latency, bandwidth, false, num_vcs);
            nodes[node1]->add_neighbor(node2);
            nodes[node2]->add_neighbor(node1);
        }
        else
        {
            // If the link exists, add the bandwidth
            // links_map[key].add_bandwidth(bandwidth);
        }
    }
    void print_links_src_dst()
    {
        auto links_map_copy = links_map;
        std::vector<std::pair<std::string, Link>> sorted_links(links_map_copy.begin(), links_map_copy.end());
        std::sort(sorted_links.begin(), sorted_links.end(),
                  [](const std::pair<std::string, Link> &a, const std::pair<std::string, Link> &b) {
                      return a.first < b.first;
                  });
        std::cout << "Keys in links_map:\n";
    
        for (const auto &entry : sorted_links)
        {
            std::cout << entry.first << std::endl;
        }
    }
    std::vector<std::string> corrupt_links(float percentage)
    {
        // Get the number of links to corrupt
        int num_links_to_corrupt = static_cast<int>(links_map.size() * percentage);
        std::vector<std::string> keys;
        for (const auto &entry : links_map)
        {
            keys.push_back(entry.first);
        }
        std::shuffle(keys.begin(), keys.end(), rng); // Shuffle the keys to randomize selection
        std::vector<std::string> corrupted_links;
        for (int i = 0; i < num_links_to_corrupt; ++i)
        {
            std::string key = keys[i];
            links_map[key].setActive(false); // Set the link to inactive
            corrupted_links.push_back(key);
        }
        return corrupted_links;
    }

    bool corrupt_links_between(int node1, int node2)
    {
        std::string key = generate_key(node1, node2);
        auto it = links_map.find(key);
        if (it != links_map.end())
        {
            it->second.setActive(false); // Set the link to inactive
            return true;
        }
        return false;
    }

    void generate_src_dst(int &num1, int &num2)
    {
        std::uniform_int_distribution<> dist(0, this->get_num_nodes() - 1);
        num1 = dist(rng);
        do
        {
            num2 = dist(rng);
        } while (num1 == num2);
    }

    virtual Link *get_link(int current, int nextnode)
    {
        std::string key = generate_key(current, nextnode);
        auto it = links_map.find(key);
        if (it != links_map.end())
        {
            return &it->second;
        }
        else
        {
            throw std::out_of_range("Link not found between nodes " + std::to_string(current) + " and " + std::to_string(nextnode));
        }
    }
    virtual std::vector<Link *> get_all_links()
    {
        std::vector<Link *> all_links;
        for (const auto &entry : links_map)
        {
            Link* link = const_cast<Link *>(&entry.second);
            if (link->getActive()) {
                all_links.push_back(link);
            }
        }
        return all_links;
    }

    virtual Node *get_node(int /*node_id*/) const
    {
        return nullptr;
    }
    virtual void reset_topology()
    {
        for (Node *node : nodes)
        {
            if (node)
            {
                node->reset_node();
            }
        }
        std::vector<Link *> links = get_all_links();
        for (Link *link : links)
        {
            if (link)
            {
                link->reset_Link();
            }
        }
    }
    // Return the physical node corresponding to the given id.
    virtual PhysicalNode get_map_node(int id) const = 0;

    // Return the distance between two physical nodes.
    virtual int distance(const PhysicalNode &a, const PhysicalNode &b) const = 0;

    // Utility to print the topology's structure
    virtual void print_topology() const = 0;
    virtual int get_Random_Node(int src, int dest) const = 0;
    virtual int get_Random_Node_In_Quadrant(const std::vector<int> &/*src_coords*/, const std::vector<int> &/*dest_coords*/) const
    {
        return 0;
    }

    // define this as a virtual function to be overriden by other classes
    virtual std::vector<Node *> get_nodes_list() const
    {
        return nodes;
    }

    // for Dragonfly
    virtual int get_nodes_per_router() const
    {
        return 0;
    }
    virtual int get_num_groups() const
    {
        return 0;
    }
    virtual int get_routers_per_group() const
    {
        return 0;
    }
    virtual int get_global_links_per_router() const
    {
        return 0;
    }

    // butterfly specific functions (implemented only in Butterfly.h)
    virtual int get_k() const
    {
        return 0;
    }
    virtual int get_n() const
    {
        return 0;
    }

    // virtual TopologyType get_topology_type() const = 0;
};
#endif
