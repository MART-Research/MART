// test_all_topologies_sa.cpp

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

// Include SA mapping algorithm.
#include "../../modules/Mapper/include/SA.hpp"

// Utility: create a complex communication matrix for 'n' virtual nodes,
// dividing them into 'num_clusters' clusters with specified intra- and inter-cluster weights.
std::vector<std::vector<double>> create_complex_comm_matrix(int n, int num_clusters,
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
    for (int i = 0; i < n; ++i)
        matrix[i][i] = 0.0;
    return matrix;
}

// Helper function to run SA mapping on a given topology.
void test_topology(const std::string& topo_name, Topology* topo, int num_virtual_nodes, int num_clusters) {
    // Build physical topology.
    topo->create_topology();
    std::cout << "SA Mapping Test on " << topo->get_topology_name() 
              << ": Physical nodes = " << topo->get_num_nodes() 
              << ", Virtual nodes = " << num_virtual_nodes << std::endl;
              
    // Create virtual nodes.
    std::vector<VirtualNode> vnodes;
    for (int i = 0; i < num_virtual_nodes; ++i) {
        vnodes.push_back({i, "VNode_" + std::to_string(i)});
    }
    
    // Generate a communication matrix.
    auto comm_matrix = create_complex_comm_matrix(num_virtual_nodes, num_clusters);
    
    // Set up SA mapping algorithm.
    SA sa;  // Uses default parameters.
    // *** FIX: set the communication matrix ***
    sa.set_comm_matrix(comm_matrix);
    
    // Compute mapping.
    std::vector<int> mapping = sa.map(vnodes, *topo);
    
    // Print the mapping result.
    std::cout << "Mapping result:" << std::endl;
    for (size_t i = 0; i < mapping.size(); ++i) {
        std::cout << "  Virtual Node " << i << " -> Physical Node " << mapping[i] << std::endl;
    }
    std::cout << "---------------------------------------" << std::endl;
}

int main() {
    // Many-to-One Mapping Tests for SA (virtual nodes > physical nodes)
    
    // Test on Mesh: A 3x3 mesh has 9 physical nodes; map 12 virtual nodes.
    {
        Mesh mesh(3, 3);
        test_topology("Mesh2D", &mesh, 12, 3);
    }
    
    // Test on Torus_2D: A 3x3 torus has 9 physical nodes; map 12 virtual nodes.
    {
        Torus torus2d(3, 3);
        test_topology("Torus2D", &torus2d, 12, 3);
    }
    
    // Test on Torus_3D: A 3x3x2 torus has 18 physical nodes; map 20 virtual nodes.
    {
        Torus torus3d(3, 3, 2);
        test_topology("Torus3D", &torus3d, 20, 4);
    }
    
    // Test on Tree: A binary tree with branching factor 2 and depth 3 (15 physical nodes) with 18 virtual nodes.
    {
        Tree tree(2, 3);
        tree.create_topology();
        test_topology("TreeTopology", &tree, 18, 3);
    }
    
    // Test on Ring: A ring with 10 physical nodes; map 15 virtual nodes.
    {
        Ring ring(10);
        test_topology("RingTopology", &ring, 15, 3);
    }
    
    // Test on Hypercube: A 4-dimensional hypercube has 16 physical nodes; map 20 virtual nodes.
    {
        Hypercube hyper(4);
        test_topology("HypercubeTopology", &hyper, 20, 3);
    }
    
    // Test on Dragonfly: For example, 4 groups with 4 nodes each (16 physical nodes) with 22 virtual nodes.
    // {
    //     Dragonfly dragonfly(4, 4);
    //     test_topology("DragonflyTopology", &dragonfly, 22, 3);
    // }
    
    // Test on Butterfly: For instance, 4 levels with 4 nodes per level (16 physical nodes) with 20 virtual nodes.
    {
        Butterfly butterfly(4, 4);
        test_topology("ButterflyTopology", &butterfly, 20, 3);
    }
    
    return 0;
}
