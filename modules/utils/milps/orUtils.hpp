#ifndef OR_UTILS_H
#define OR_UTILS_H

#include "MARTUtility.hpp"
#include "absl/flags/flag.h"
#include "absl/log/flags.h"
#include "absl/time/time.h"
#include "ortools/base/init_google.h"
#include "ortools/base/logging.h"
#include "ortools/init/init.h"
#include "ortools/linear_solver/linear_solver.h"
namespace operations_research {
namespace fs = std::filesystem;

    void WriteVariablesToFile(const operations_research::MPSolver& solver, const std::string& filepath, const std::string mode = "append") {
        fs::path path(filepath);
        fs::path directory = path.parent_path();
        if (!fs::exists(directory)) { fs::create_directories(directory); }
        std::ofstream file;
        if (mode == "append") 
            file.open(filepath, std::ios::app);
        else 
            file.open(filepath);

        if (!file) { std::cerr << "Error opening file: " << filepath << "\n"; return; }

        if (mode == "append") file << "\n\n\n";

        file << "# ---------- Variables ----------\n";
        for (int i = 0; i < solver.NumVariables(); ++i) {
            const auto* variable = solver.variable(i);
            file << variable->name() << " = " << variable->solution_value() << " "
                << "in [" << variable->lb() << ", " << variable->ub() << "]\n";
        }
        file.close();
    }

    void writeConstraintsToFile(const operations_research::MPSolver& solver, const std::string& filepath, const std::string mode = "append") {
        fs::path path(filepath);
        fs::path directory = path.parent_path();
        if (!fs::exists(directory)) { fs::create_directories(directory); }
        std::ofstream file;
        if (mode == "append") 
            file.open(filepath, std::ios::app);
        else 
            file.open(filepath);

        if (!file) { std::cerr << "Error opening file: " << filepath << "\n"; return; }

        if (mode == "append") file << "\n\n\n";

        file << "# ---------- Constraints ----------\n";
        for (int i = 0; i < solver.NumConstraints(); ++i) {
            const auto* constraint = solver.constraint(i);
            file << constraint->name() << " ---- ";
            for (const auto& term : constraint->terms()) { file << term.second << " * " << term.first->name() << " "; }
            file << " in [" << constraint->lb() << ", " << constraint->ub() << "]\n";
        }
        file.close();
    }



    void displayConstraints(const operations_research::MPSolver& solver) {
        for (int i = 0; i < solver.NumConstraints(); ++i) {
            const auto* constraint = solver.constraint(i);
            std::cout << "Constraint " << i + 1 << ": " << constraint->name() << "  ----  ";
            for (const auto& term : constraint->terms())
                std::cout << term.second << " * " << term.first->name() << " ";
            std::cout << " in [" << constraint->lb() << ", " << constraint->ub() << "]\n";
        }
    }
    
    void displayVariables(const operations_research::MPSolver& solver) {
        for (int i = 0; i < solver.NumVariables(); ++i) {
            const auto* variable = solver.variable(i);
            std::cout << "Variable " << i + 1 << ": " << variable->name() << " = " << variable->solution_value() << " "
                      << "in [" << variable->lb() << ", " << variable->ub() << "]\n";
        }
    }
    
    // generateTraverseibleEdges (used for algorithms operating on CDGs, i.e. BSOR)
    // A DFS function that generates the traversible edges for which flow variables are generated
    // For each flow, run a dfs to get all the traversible edges starting at si and ending at ti
    // create the variables fi and bi over all the obtained edges
    bool generateTraversibleEdges (const std::pair<int, int> flowPair, std::pair<int, int> vertex, 
    const std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash>& flowNetwork, 
    std::unordered_map<std::pair<int, int>, bool, pair_hash> &isVisited, std::unordered_map<std::pair<int, int>, bool, pair_hash> &leadsToTi,
    std::vector<std::tuple<int, int, int, int>>& TraversibleEdges) {
        if (leadsToTi[vertex]) return true;
        if (vertex == std::make_pair(flowPair.second, 0)) return leadsToTi[vertex] = true;
        if (isVisited[vertex]) return false;
        isVisited[vertex] = true;
        auto currentVertexNeighborVertices = flowNetwork.find(vertex);
        if (currentVertexNeighborVertices == flowNetwork.end()) return false;
        bool hasatLeastOnePathtoTi = false;
        for (const auto& Neighbor : currentVertexNeighborVertices->second) {
            const int &p2_1 = Neighbor.first, &p2_2 = Neighbor.second;
            if (generateTraversibleEdges(flowPair, {p2_1, p2_2}, flowNetwork, isVisited, leadsToTi, TraversibleEdges)) {
                TraversibleEdges.push_back(std::make_tuple(vertex.first, vertex.second, p2_1, p2_2));
                leadsToTi[vertex] = true;
                hasatLeastOnePathtoTi = true;
            }
        }
        return hasatLeastOnePathtoTi;
    }

    // generateTraverseibleEdges (used for algorithms operating on normal flow graphs, i.e. Transcom)
    // A DFS function that generates the traversible edges for which flow variables are generated
    // For each flow, run a dfs to get all the traversible edges starting at si and ending at ti
    // create the variables fi and bi over all the obtained edges
    bool generateTraversibleEdges (const int destination, int vertex, const std::unordered_map<int, std::vector<int>> &flowNetwork, 
    std::unordered_map<int, bool> &isVisited, std::unordered_map<int, bool> &leadsToTi,
    std::vector<std::pair<int, int>>& TraversibleEdges) 
    {
        if (leadsToTi[vertex]) return true;
        if (vertex == destination) return leadsToTi[vertex] = true;
        if (isVisited[vertex]) return false;
        isVisited[vertex] = true;
        auto currentVertexNeighborVertices = flowNetwork.find(vertex);
        if (currentVertexNeighborVertices == flowNetwork.end()) return false;
        bool hasatLeastOnePathtoTi = false;
        for (const auto& neighbor : currentVertexNeighborVertices->second) {
            if (generateTraversibleEdges(destination, neighbor, flowNetwork, isVisited, leadsToTi, TraversibleEdges)) {
                TraversibleEdges.push_back({vertex, neighbor});
                leadsToTi[vertex] = true;
                hasatLeastOnePathtoTi = true;
            }
        }
        // isVisited[vertex] = false;
        return hasatLeastOnePathtoTi;
    }
    
    bool generateTraversibleEdgesBFS(int  source, int  /*destination but will neer be used*/, const std::unordered_map<int,std::vector<int>>& flowNetwork, std::unordered_map<int,bool>& leadsToTi, std::vector<std::pair<int,int>>& traversibleEdges)
    {
        traversibleEdges.clear();
        leadsToTi.clear();

        if (flowNetwork.empty())
            return false;

        std::unordered_map<int,std::vector<int>> adj(flowNetwork);
        for (const auto& [u, nbrs] : flowNetwork)
            for (int v : nbrs) adj[v].push_back(u);
        std::queue<int> q;
        q.push(source);
        leadsToTi[source] = true;

        // To prevent inserting the same oriented edge twice
        std::unordered_set<long long> seen;
        auto encode = [](int a,int b)->long long {
            return (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
        };

        while (!q.empty()) {
            int u = q.front(); q.pop();

            for (int v : adj[u]) {
                long long fwd = encode(u,v), rev = encode(v,u);
                if (!seen.count(fwd)) {
                    traversibleEdges.emplace_back(u,v);
                    seen.insert(fwd);
                }
                if (!seen.count(rev)) {
                    traversibleEdges.emplace_back(v,u);
                    seen.insert(rev);
                }

                if (!leadsToTi[v]) {
                    leadsToTi[v] = true;
                    q.push(v);
                }
            }
        }

        return !traversibleEdges.empty();
    }

    std::vector<std::pair<int, int>> getAllEdges(const std::unordered_map<int, std::vector<int>>& flowNetwork) {
        std::vector<std::pair<int, int>> edges;
    
        for (const auto& [src, neighbors] : flowNetwork) {
            for (const int& dst : neighbors) {
                edges.emplace_back(src, dst);
            }
        }
        return edges;
    }

    std::vector<std::tuple<int, int, int, int>> getAllEdges(const std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash>& flowNetwork)
    {
        std::vector<std::tuple<int, int, int, int>> TraversibleEdges;
        for (const auto& [src, neighbors] : flowNetwork) {
            for (const auto& dst : neighbors) {
                TraversibleEdges.emplace_back(src.first, src.second, dst.first, dst.second);
            }
        }
        return TraversibleEdges;
    }

    void writeLogToFile(std::stringstream& ss, const std::string& filepath) {
        fs::path path(filepath);
        fs::path directory = path.parent_path();
        if (!fs::exists(directory)) { fs::create_directories(directory); }
        std::ofstream file(filepath);
        if (!file) { std::cerr << "Error opening file: " << filepath << "\n"; return; }

        file << "## ----- Solver Log and Important Info -----\n";
        file << ss.str();

        file.close();
    }
}


#endif // OR_UTILS_H

