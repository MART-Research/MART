// test_all_topologies_ga.cpp

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

// Include GA mapping algorithm.
#include "../../modules/Mapper/include/GA.hpp"

// Utility: create a communication matrix for 'n' virtual nodes,
// dividing them into 'num_clusters' clusters with given intra- and inter-cluster weights.
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

// Helper function to run GA mapping on a given topology.
void test_topology(const std::string& topo_name, Topology* topo, int num_virtual_nodes, int num_clusters) {
    // Build the physical topology.
    topo->create_topology();
    std::cout << "GA Mapping Test on " << topo->get_topology_name() 
              << ": Physical nodes = " << topo->get_num_nodes() 
              << ", Virtual nodes = " << num_virtual_nodes << std::endl;
    
    // Create virtual nodes.
    std::vector<VirtualNode> vnodes;
    for (int i = 0; i < num_virtual_nodes; i++) {
        vnodes.push_back({i, "VNode_" + std::to_string(i)});
    }
    
    // Generate a communication matrix.
    auto comm_matrix = create_complex_comm_matrix(num_virtual_nodes, num_clusters);
    
    // Set up GA mapping algorithm.
    // You can adjust population_size, generations, crossover_rate, mutation_rate, and overload_penalty as needed.
    GA ga(50, 500, 0.8, 0.1, 10.0);
    ga.set_comm_matrix(comm_matrix);
    
    // Compute mapping.
    std::vector<int> mapping = ga.map(vnodes, *topo);
    
    // Print mapping result.
    std::cout << "Mapping result:" << std::endl;
    for (size_t i = 0; i < mapping.size(); i++) {
        std::cout << "  Virtual Node " << i << " -> Physical Node " << mapping[i] << std::endl;
    }
    std::cout << "---------------------------------------" << std::endl;
}

int main() {
    // For GA, we choose many-to-one scenarios (virtual nodes > physical nodes).

    // Test on Mesh2D: For example, a 3x3 mesh (9 physical nodes) with 12 virtual nodes.
    {
        Mesh mesh(3, 3);
        test_topology("Mesh2D", &mesh, 12, 3);
    }
    
    // Test on Torus2D: 3x3 torus (9 physical nodes) with 12 virtual nodes.
    {
        Torus torus2d(3, 3);
        test_topology("Torus2D", &torus2d, 12, 3);
    }
    
    // Test on Torus3D: 3x3x2 torus (18 physical nodes) with 20 virtual nodes.
    {
        Torus torus3d(3, 3, 2);
        test_topology("Torus3D", &torus3d, 20, 4);
    }
    
    // Test on TreeTopology: For example, a binary tree with branching factor 2 and depth 3 (15 physical nodes) with 18 virtual nodes.
    {
        Tree tree(2, 3);
        tree.create_topology(); // Ensure tree is built.
        test_topology("TreeTopology", &tree, 18, 3);
    }
    
    // Test on RingTopology: For example, a ring with 10 physical nodes with 15 virtual nodes.
    {
        Ring ring(10);
        test_topology("RingTopology", &ring, 15, 3);
    }
    
    // Test on HypercubeTopology: For example, a 4-dimensional hypercube (16 physical nodes) with 20 virtual nodes.
    {
        Hypercube hyper(4);
        test_topology("HypercubeTopology", &hyper, 20, 3);
    }
    
    // Test on DragonflyTopology: For example, 4 groups with 4 nodes each (16 physical nodes) with 22 virtual nodes.
    // {
    //     Dragonfly dragonfly(4, 4);
    //     test_topology("DragonflyTopology", &dragonfly, 22, 3);
    // }
    
    // Test on ButterflyTopology: For example, 4 levels with 4 nodes per level (16 physical nodes) with 20 virtual nodes.
    {
        Butterfly butterfly(4, 4);
        test_topology("ButterflyTopology", &butterfly, 20, 3);
    }
    
    return 0;
}
