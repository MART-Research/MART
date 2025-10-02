// test_all_topologies_bnb.cpp

#include <iostream>
#include <vector>
#include <tuple>
#include <map>
#include <utility>
#include <algorithm>

// Include common node structure and topology interface.
#include "../../modules/utils/nodes/map_nodes.hpp"
#include "../../modules/utils/networks/Topology.hpp"

// Include all topologies.
#include "../../modules/utils/networks/topologies/Butterfly.hpp"
#include "../../modules/utils/networks/topologies/Dragonfly.hpp"
#include "../../modules/utils/networks/topologies/Hypercube.hpp"
#include "../../modules/utils/networks/topologies/Mesh.hpp"
#include "../../modules/utils/networks/topologies/Ring.hpp"
#include "../../modules/utils/networks/topologies/Torus.hpp"
#include "../../modules/utils/networks/topologies/Tree.hpp"

// Include BnB mapping algorithm.
#include "../../modules/Mapper/include/BnB.hpp"

// Utility: create a simple communication matrix for n virtual nodes,
// dividing them into num_clusters clusters.
// (Because BnB is exponential, we use small n.)
std::vector<std::vector<double>> create_simple_comm_matrix(int n, int num_clusters,
                                                           double intra_cluster_weight = 5.0,
                                                           double inter_cluster_weight = 0.5) {
    std::vector<std::vector<double>> matrix(n, std::vector<double>(n, inter_cluster_weight));
    int cluster_size = (num_clusters > 0) ? (n / num_clusters) : n;
    for (int cluster = 0; cluster < num_clusters; ++cluster) {
        int start = cluster * cluster_size;
        int end = (cluster == num_clusters - 1) ? n : start + cluster_size;
        for (int i = start; i < end; ++i) {
            for (int j = i + 1; j < end; ++j) {
                matrix[i][j] = intra_cluster_weight;
                matrix[j][i] = intra_cluster_weight;
            }
        }
    }
    // Set self communication to zero.
    for (int i = 0; i < n; ++i)
        matrix[i][i] = 0.0;
    return matrix;
}

// Helper function to run BnB mapping on a given topology.
void test_topology(const std::string& topo_name, Topology* topo, int num_virtual_nodes, int num_clusters) {
    // Build physical topology.
    topo->create_topology();
    std::cout << "BnB Mapping Test on " << topo->get_topology_name() 
              << ": Physical nodes = " << topo->get_num_nodes() 
              << ", Virtual nodes = " << num_virtual_nodes << std::endl;
    
    // Create a list of virtual nodes.
    std::vector<VirtualNode> vnodes;
    for (int i = 0; i < num_virtual_nodes; ++i) {
        vnodes.push_back({i, "VNode_" + std::to_string(i)});
    }
    
    // Generate a simple communication matrix.
    auto comm_matrix = create_simple_comm_matrix(num_virtual_nodes, num_clusters);
    
    // Set up BnB mapping instance, for example with overload_penalty of 10.0.
    BnB bnb(10.0);
    bnb.set_comm_matrix(comm_matrix);
    
    // Compute mapping.
    std::vector<int> mapping = bnb.map(vnodes, *topo);
    
    // Print mapping result.
    std::cout << "Mapping result:" << std::endl;
    for (size_t i = 0; i < mapping.size(); ++i) {
        std::cout << "  Virtual Node " << i << " -> Physical Node " << mapping[i] << std::endl;
    }
    
    std::cout << "---------------------------------------" << std::endl;
}

int main() {
    // Because branch-and-bound examines the entire search space, we need small n.
    // But now we require many-to-one (virtual nodes > physical nodes).
    
    // Test on Mesh (Mesh2D): A 2x2 mesh has 4 physical nodes; map 5 virtual nodes.
    {
        Mesh mesh(2, 2);
        test_topology("Mesh2D", &mesh, 5, 2);
    }
    
    // Test on Torus_2D: A 2x2 torus (4 physical nodes); map 5 virtual nodes.
    {
        Torus torus2d(2, 2);
        test_topology("Torus2D", &torus2d, 5, 2);
    }
    
    // // Test on Torus_3D: A 2x2x1 torus (4 physical nodes); map 5 virtual nodes.
    // {
    //     Torus torus3d(2, 2, 1);
    //     test_topology("Torus3D", &torus3d, 5, 2);
    // }
    
    // Test on Tree (TreeTopology): A binary tree with branching factor 2 and depth 2 produces 1+2+4 = 7 physical nodes; map 8 virtual nodes.
    {
        Tree tree(2, 2);
        tree.create_topology();  // Ensure tree is built.
        test_topology("TreeTopology", &tree, 8, 2);
    }
    
    // Test on Ring (RingTopology): A ring with 5 physical nodes; map 6 virtual nodes.
    {
        Ring ring(5);
        test_topology("RingTopology", &ring, 6, 2);
    }
    
    // Test on Hypercube (HypercubeTopology): A 3-dimensional hypercube has 8 physical nodes; map 9 virtual nodes.
    {
        Hypercube hyper(2);
        test_topology("HypercubeTopology", &hyper, 5, 2);
    }
    
    // Test on Dragonfly (DragonflyTopology): For example, 2 groups with 3 nodes each (6 physical nodes); map 7 virtual nodes.
    // {
    //     Dragonfly dragonfly(2, 3);
    //     test_topology("DragonflyTopology", &dragonfly, 7, 2);
    // }
    
    // Test on Butterfly (ButterflyTopology): For instance, 3 levels with 3 nodes per level (9 physical nodes); map 10 virtual nodes.
    {
        Butterfly butterfly(2, 3);
        test_topology("ButterflyTopology", &butterfly, 8, 2);
    }
    
    return 0;
}
