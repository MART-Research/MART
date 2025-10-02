/*
// Utility functions and declarations needed for the tool
*/
#ifndef MART_UTILITY_H
#define MART_UTILITY_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <utility>
#include <functional>
#include <cstdlib>
#include <memory>
#include <limits>
#include <algorithm>
#include <cmath>
#include <vector>
#include <queue>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <queue>

namespace fs = std::filesystem;

const int INF = std::numeric_limits<int>::max();

// Custom hash function for std::pair
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};


std::vector<std::pair<int, int>> extractKeys(const std::unordered_map<std::pair<int, int>, int, pair_hash>& edges) {
    std::vector<std::pair<int, int>> keys;
    keys.reserve(edges.size());
    for (const auto& entry : edges) {
        keys.push_back(entry.first);
    }
    return keys;
}

void printCDG(const std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash>& cdg) {
    for (const auto& entry : cdg) {
        const auto& key = entry.first;
        const auto& value = entry.second;
        std::cout << "Edge: (" << key.first << ", " << key.second << ") -> [";

        for (size_t i = 0; i < value.size(); ++i) {
            const auto& edge = value[i];
            std::cout << "(" << edge.first << ", " << edge.second << ")";
            if (i < value.size() - 1) {
                std::cout << ", ";
            }
        }
        
        std::cout << "]\n";
    }
}

// Functions for calculating shortest paths for hop values
using EdgeList = std::vector<std::pair<int, int>>;
using AdjacencyList = std::unordered_map<int, std::vector<int>>;
using PairList = std::vector<std::pair<int, int>>;

EdgeList extractEdgeList(const std::unordered_map<int, std::vector<int>>& flowNetwork) {
    EdgeList edgeList;
    for (const auto& entry : flowNetwork) {
        int u = entry.first; // The key node
        const auto& neighbors = entry.second; // Connected nodes (vector)
        for (const auto& v : neighbors) {
            edgeList.push_back({u, v}); // Add the edge (u, v) to the edge list
        }
    }
    return edgeList;
}


// Converts edge list to adjacency list
AdjacencyList convertToAdjacencyList(const EdgeList& edgeList) {
    AdjacencyList adjList;
    for (const auto& edge : edgeList) {
        adjList[edge.first].push_back(edge.second);
    }
    return adjList;
}

// Performs BFS to find shortest path from start node to target node
int bfsShortestPath(const AdjacencyList& graph, int start, int target) {
    std::queue<int> q;
    std::unordered_map<int, int> distances;

    q.push(start);
    distances[start] = 0;

    while (!q.empty()) {
        int node = q.front();
        q.pop();

        if (node == target) {
            return distances[node];
        }

        auto it = graph.find(node);
        if (it != graph.end()) {
            for (int neighbor : it->second) {
                if (distances.find(neighbor) == distances.end()) {
                    distances[neighbor] = distances[node] + 1;
                    q.push(neighbor);
                }
            }
        }
    }

    return INF;
}

// Finds shortest paths for specific pairs
std::unordered_map<std::pair<int, int>, int, pair_hash> shortestPathsForPairs(const EdgeList& edgeList, const PairList& pairs, int numNodes) {
    AdjacencyList adjList = convertToAdjacencyList(edgeList);
    std::unordered_map<std::pair<int, int>, int, pair_hash> results;
    for (const auto& pair : pairs) {
        int start = pair.first;
        int target = pair.second;
        int shortestPath = bfsShortestPath(adjList, start, target);
        results[{start, target}] = shortestPath; // Store the result in the map
    }
    return results;
}

// Function to print the hop map
void printHopMap(const std::unordered_map<std::pair<int, int>, int, pair_hash>& hop) {
    std::cout << "Hop distances between nodes:\n";
    std::cout << "---------------------------\n";
    for (const auto& entry : hop) {
        const auto& key = entry.first;
        const auto& value = entry.second;
        std::cout << "(" << key.first << ", " << key.second << ") : " << value << "\n";
    }
    std::cout << "---------------------------\n";
}

// Function to extract filename from the given path
std::string extract_filename(const std::string& filepath) {
    return fs::path(filepath).filename().string();
}

// i think all should use double for weights
std::multimap<int, std::pair<int, double>> readFlows_singledest_double(const std::string& filepath) {
    std::multimap<int, std::pair<int, double>> flows;

    
    std::string filename = extract_filename(filepath);

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open flows file: " << filepath << "\n";
        return flows;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double weight;
        int src, dst;
        if (!(iss >> src >> dst >> weight)) continue;
        flows.insert({src, {dst, weight}});
    }

    file.close();
    return flows;
}

std::multimap<int, std::pair<int, int>> readFlows_singledest(const std::string& filepath) {
    std::multimap<int, std::pair<int, int>> flows;

    // Ensure we use the correct filename
    std::string filename = extract_filename(filepath);

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open flows file: " << filepath << "\n";
        return flows;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int weight, src, dst;
        if (!(iss >> src >> dst >> weight)) continue;  // Skip malformed lines
        flows.insert({src, {dst, weight}});

        // std::cout << "Flow data = " << "src = " << src << ", dst = " << dst << ", weight = " << weight << std::endl;
    }

    file.close();
    return flows;
}

// **Multi Destination Flow Function**
std::unordered_map<int, std::pair<std::unordered_set<int>, double>> readFlows_multidest(const std::string& filepath) {
    std::unordered_map<int, std::pair<std::unordered_set<int>, double>> flows;
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open flows file: " << filepath << "\n";
        return flows;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int src, dst;
        std::unordered_set<int> destinations;
        while (iss >> dst && iss.peek() != ' ') {
            destinations.insert(dst);
        }
        double weight;
        iss >> weight;
        
        flows[src] = {destinations, weight};
    }

    file.close();
    return flows;
}


void printCommunicationGraph(const std::multimap<int, std::pair<int, int>>& communication_graph) {
    for (const auto& pair : communication_graph) {
        std::cout << "src = " << pair.first << ", dst = " << pair.second.first << ", weight = " << pair.second.second << "\n";
    }
}

std::unordered_map<int, std::vector<int>> readFlowNetwork(const std::string& filepath) {
    // std::cout << "Current Directory: " << fs::current_path() << std::endl;
    std::unordered_map<int, std::vector<int>> network;
    
    std::string filename = extract_filename(filepath);

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open flow network file: " << filepath << "\n";
        return network;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int src;
        if (!(iss >> src)) continue;
        
        int dst;
        while (iss >> dst) {
            network[src].push_back(dst);
        }
    }

    file.close();
    return network;
}

void printFlowNetwork(const std::unordered_map<int, std::vector<int>>& flowNetwork) {
    std::cout << "\nFlow Network Contents:\n";
    for (const auto& [node, neighbors] : flowNetwork) {
        std::cout << node << ": ";
        for (int neighbor : neighbors) {
            std::cout << neighbor << " ";
        }
        std::cout << "\n";
    }
}



std::unordered_map<std::pair<int, int>, int, pair_hash> readFlowNetwork_Capacity(const std::string& filepath) {
    // std::cout << "Current Directory: " << fs::current_path() << std::endl;
    std::unordered_map<std::pair<int, int>, int, pair_hash> network;

    // Ensure we use the correct filename from the path
    std::string filename = extract_filename(filepath);

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open flow network file: " << filepath << "\n";
        return network;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int src, dst, capacity;
        iss >> src >> dst >> capacity;
        network[{src, dst}] = capacity;
    }

    file.close();
    return network;
}

void printFlowNetwork_Capacity(const std::unordered_map<std::pair<int, int>, int, pair_hash>& flowNetwork) {
    std::cout << "\nFlow Network Contents:\n";
    for (const auto& edge : flowNetwork) {
        std::cout << "Edge: (" << edge.first.first << ", " << edge.first.second << ") Capacity: " << edge.second << "\n";
    }
}


template <typename WeightT = int>
std::multimap<int, std::pair<int, WeightT>> retrive_physical_mapping( const std::multimap<int, std::pair<int, WeightT>> &flows, const std::vector<int> &mapping) {
    // First stage: accumulate with a unique key (physSrc, physDst)
    std::map<std::pair<int, int>, WeightT> agg;

    for (const auto &e : flows) {
        int   logical_src = e.first;
        int   logical_dst = e.second.first;
        WeightT weight    = e.second.second;
        if (logical_src == logical_dst || weight == WeightT{}) continue;
        int phys_src = mapping.at(logical_src);
        int phys_dst = mapping.at(logical_dst);
        if (phys_src == phys_dst) continue;
        agg[{phys_src, phys_dst}] += weight;
    }

    // Second stage: copy into the requested *multimap* shape
    std::multimap<int, std::pair<int, WeightT>> result;
    for (auto &kv : agg) result.emplace(kv.first.first, std::make_pair(kv.first.second, kv.second));
    return result;
}



template <typename W = int>
std::unordered_map<int,std::pair<std::unordered_set<int>,W>> retrive_physical_mapping( const std::unordered_map<int, std::pair<std::unordered_set<int>, W>> &flows, const std::vector<int> &mapping)
{
    using DestSet = std::unordered_set<int>;
    std::unordered_map<int, std::pair<DestSet, W>> result;

    for (const auto &entry : flows)
    {
        int logical_src          = entry.first;
        const DestSet &dst_set   = entry.second.first;
        W weight                 = entry.second.second;

        int phys_src = mapping.at(logical_src);
        auto &bucket = result[phys_src];

        if (bucket.second != W{} && bucket.second != weight)
            throw std::logic_error( "different weights mapped to the same physical source node");
        bucket.second = weight;

        
        for (int logical_dst : dst_set)
        {
            if (logical_dst == logical_src) continue;
            int phys_dst = mapping.at(logical_dst);
            if (phys_dst == phys_src) continue;
            bucket.first.insert(phys_dst);
        }
    }

    
    for (auto it = result.begin(); it != result.end(); )
        it = it->second.first.empty() ? result.erase(it) : std::next(it);

    return result;
}


// General Utility
// Helper function to determine the actual input file to parse
std::string determineActualInputFile(const std::string& initial_path_str) {
    fs::path initial_path(initial_path_str);
    std::string file_to_parse_str;

    if (!fs::exists(initial_path)) {
        throw std::runtime_error("Error: Path does not exist: " + initial_path_str);
    }

    if (fs::is_directory(initial_path)) {
        fs::path last_file_path;
        std::filesystem::file_time_type last_write_time;
        bool found_file = false;

        for (const auto& entry : fs::directory_iterator(initial_path)) {
            if (fs::is_regular_file(entry.path())) {
                auto current_file_time = fs::last_write_time(entry.path());
                if (!found_file || current_file_time > last_write_time) {
                    last_write_time = current_file_time;
                    last_file_path = entry.path();
                    found_file = true;
                }
            }
        }
        if (!found_file) {
             throw std::runtime_error("Error: Directory contains no regular files: " + initial_path_str);
        }
        file_to_parse_str = last_file_path.string();
        std::cout << "Info: Parsing last modified file in directory: " << file_to_parse_str << std::endl;
    } else if (fs::is_regular_file(initial_path)) {
        file_to_parse_str = initial_path_str;
    } else {
        throw std::runtime_error("Error: Path is not a regular file or directory: " + initial_path_str);
    }
    return file_to_parse_str;
}

#endif // MART_UTILITY_H
