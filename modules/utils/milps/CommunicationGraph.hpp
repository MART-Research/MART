#ifndef COMMUNICATION_GRAPH_H
#define COMMUNICATION_GRAPH_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <utility>
class CommunicationGraph {
public:
    CommunicationGraph() {}
    CommunicationGraph(int num_nodes) : num_nodes(num_nodes) {
        // flows_adj_matrix.resize(num_nodes, std::vector<int>(num_nodes, 0));

        buildCommGraph();
    }

    void add_flow (int from, int to, int demand) {
        flows.insert({from, {to, demand}});
    }

    void buildCommGraph () {
        // TODO
        // This is an input
        // should be read from a file
        
        // for now, we assume a fixed communication graph over a fixed topology; the one described in the paper and CDG.h
        flows.insert({6, {0, 2}});  // A -> L with demand 2
        flows.insert({4, {2, 5}});  // E -> G with demand 5
    }

    int get_num_nodes() const { return num_nodes; }

    std::multimap<int, std::pair<int, int>> getCommGraph() const {
        return flows;
    }
    
private:
    int num_nodes;
    // std::vector<std::pair<int, std::pair<int, int>>> flows;  // I believe we should represent the flows as an edge list or adjacency list
    std::multimap<int, std::pair<int, int>> flows;
};

#endif


/*

// Adjacency Matrix Representation - related functions and declarations

// std::vector<std::vector<int>> flows_adj_matrix;

// void add_edge(int from, int to, int demand) {
//     if (from < num_nodes && to < num_nodes) {
//         flows_adj_matrix[from][to] = demand;       // directed graph
//         // flows_adj_matrix[to][from] = 1;  // Assuming undirected graph
//     }
// }

// void print_graph() const {
//     std::cout << "Communication Graph Adjacency Matrix:\n";
//     for (const auto& row : flows_adj_matrix) {
//         for (int val : row) {
//             std::cout << val << " ";
//         }
//         std::cout << std::endl;
//     }
// }


// std::vector<std::vector<int>> getCommGraph () const {
//     return flows_adj_matrix;
// }

*/