// test_all_topologies_aco.cpp
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

// Include ACO mapping algorithm.
#include "../../modules/Mapper/include/ACO.hpp"

// Utility: create a complex communication matrix for 'n' virtual nodes,
// dividing them into 'num_clusters' clusters.
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

// Updated aggregate function (as provided above).
std::vector<std::tuple<int, int, double>> aggregate_communication(
    const std::vector<std::vector<double>>& comm_matrix,
    const std::vector<int>& mapping)
{
    int n = mapping.size();
    std::map<std::pair<int, int>, double> agg;
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if(i == j) continue;
            int phys_i = mapping[i];
            int phys_j = mapping[j];
            if (phys_i == phys_j) continue;
            std::pair<int, int> edge = {phys_i, phys_j};
            agg[edge] += comm_matrix[i][j];
        }
    }
    
    std::vector<std::tuple<int, int, double>> aggregated;
    for (auto& kv : agg) {
        aggregated.push_back(std::make_tuple(kv.first.first, kv.first.second, kv.second));
    }
    
    return aggregated;
}

// Helper function to run ACO mapping on a given topology.
void test_topology(const std::string& topo_name, Topology* topo, int num_virtual_nodes, int num_clusters) {
    topo->create_topology();
    std::cout << "ACO Mapping Test on " << topo->get_topology_name()
              << ": Physical nodes = " << topo->get_num_nodes()
              << ", Virtual nodes = " << num_virtual_nodes << std::endl;
              
    // Create virtual nodes.
    std::vector<VirtualNode> vnodes;
    for (int i = 0; i < num_virtual_nodes; ++i) {
        vnodes.push_back({i, "VNode_" + std::to_string(i)});
    }
    
    // Create communication matrix.
    auto comm_matrix = create_complex_comm_matrix(num_virtual_nodes, num_clusters);
    
    // Set up and run ACO mapping.
    ACO aco;
    aco.set_comm_matrix(comm_matrix);
    std::vector<int> mapping = aco.map(vnodes, *topo);
    
    // Print mapping result.
    std::cout << "Mapping result:" << std::endl;
    for (size_t i = 0; i < mapping.size(); ++i) {
        std::cout << "  Virtual Node " << i << " -> Physical Node " << mapping[i] << std::endl;
    }
    
    // Aggregate virtual communication into directed physical communication.
    auto aggregated = aggregate_communication(comm_matrix, mapping);
    std::cout << "\nAggregated Communication among Physical Nodes:" << std::endl;
    for (auto& tup : aggregated) {
        int src, dst;
        double weight;
        std::tie(src, dst, weight) = tup;
        std::cout << "  Physical Node " << src << " -> " << dst << " : " << weight << std::endl;
    }
    
    std::cout << "---------------------------------------" << std::endl;
}

int main() {
    // Run tests with ACO on various topologies.
    
    // Mesh2D
    {
        Mesh mesh(3, 3);
        test_topology("Mesh2D", &mesh, 12, 3);
    }
    
    // Torus2D
    {
        Torus torus2d(3, 3);
        test_topology("Torus2D", &torus2d, 12, 3);
    }
    
    // Torus3D
    {
        Torus torus3d(3, 3, 2);
        test_topology("Torus3D", &torus3d, 20, 4);
    }
    
    // TreeTopology
    {
        Tree tree(2, 3);
        tree.create_topology();
        test_topology("TreeTopology", &tree, 18, 3);
    }
    
    // RingTopology
    {
        Ring ring(10);
        test_topology("RingTopology", &ring, 15, 3);
    }
    
    // HypercubeTopology
    {
        Hypercube hyper(4);
        test_topology("HypercubeTopology", &hyper, 20, 3);
    }
    
    // DragonflyTopology
    // {
    //     Dragonfly dragonfly(4, 4);
    //     test_topology("DragonflyTopology", &dragonfly, 22, 3);
    // }
    
    // ButterflyTopology
    {
        Butterfly butterfly(4, 4);
        test_topology("ButterflyTopology", &butterfly, 20, 3);
    }
    
    return 0;
}
