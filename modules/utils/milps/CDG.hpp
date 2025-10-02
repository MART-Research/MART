#include <iostream>
#include "MARTUtility.hpp"
#include <unordered_map>
#include <utility>
#include <functional>
#include <vector>
#include "TurnModel.hpp"

class CDG {
public:

    CDG(const std::vector<std::pair<int, int>>& edges) : turnModelChecker_(NULL), flow_graph_edges(edges) {} // this is the one we are mainly working with

    CDG(TurnModel* turnModel, const std::vector<std::pair<int, int>>& edges) : turnModelChecker_(turnModel), flow_graph_edges(edges) {}

    void setTurnModel(TurnModel* turnModel) {
        turnModelChecker_ .set_strategy(turnModel);
    }
    
    void createCDG() {
        for (const auto& edge : flow_graph_edges) {
            std::vector<std::pair<int, int>> consecutive_edges;
            for (const auto& other_edge : flow_graph_edges) {
                if (edge.second == other_edge.first && edge.first != other_edge.second) {
                    consecutive_edges.push_back(other_edge);
                }
            }
            cdg[edge] = consecutive_edges;
        }
    }

    std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash> createACDG(int cols) {
        for (const auto& entry : cdg) {
            const auto& edge = entry.first;
            const auto& consecutive_edges = entry.second;

            std::vector<std::pair<int, int>> valid_edges;

            for (const auto& consecutive_edge : consecutive_edges) {
                if (!turnModelChecker_.check_violation(edge, consecutive_edge, cols)) {
                    valid_edges.push_back(consecutive_edge);
                }
            }

            if (!valid_edges.empty()) {
                a_cdg[edge] = valid_edges;
            }
        }
        return a_cdg;
    }

    auto getCDG() const {
        return cdg;
    }

    auto getACDG() const {
        return a_cdg;
    }

private:

    // used to check the turn model for creating a correpsonding ACDG
    TurnModelChecker turnModelChecker_;
    
    // Edge List representing the flow graph as retrieved from the communication graph
    std::vector<std::pair<int, int>> flow_graph_edges;
    
    // Each element (link) in the CDG is represented by two nodes forming a channel link, in addition to a vector of other links which can be traversed from that link
    std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash> cdg;

    // ACDG is the acyclic CDG, created from the CDG after applying a turn model
    std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash> a_cdg;
};