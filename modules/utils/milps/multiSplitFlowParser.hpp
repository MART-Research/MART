#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <stdexcept>
#include <set>
#include <utility>
#include <limits>
#include <algorithm>
#include <filesystem>
#include <cmath>
#include <iomanip>
#include "MARTUtility.hpp"

namespace fs = std::filesystem;

using Path = std::vector<int>;

// Structure to hold parsed results
struct multiSplitFlowData {
    unsigned int numThreads;
    std::map<int, std::map<int, Path>> routes;                          // routes[flow_id][split_id] = path (vector of node ids)
    std::map<int, std::map<int, double>> flow_split_weights;            // flow_split_weights[flow_id][split_id] = weight of that specific split
    std::map<std::pair<int, int>, double> edge_loads;                   // edge_loads[{u, v}] = total flow amount on edge (u, v)
    long long total_hops = 0;                                           // total_hops = sum of hops across all paths
    std::map<int, double> objectives;                                   // vector of objectives (e.g., objectives[1] for O1, objectives[2] for O2)
    std::map<int, double> total_flow_weights;                           // total_flow_weights[flow_id] = sum of weights of all splits for that flow
    double total_power;
    std::vector<int> mapping;
};

// Helper function to parse the variable name like "f_i_j_u_v" or "b_i_j_u_v"
bool parseVarName_multiSplit(const std::string& var_name, int& i, int& j, int& u, int& v) {
    std::stringstream ss(var_name);
    std::string segment;

    if (!(ss.ignore(std::numeric_limits<std::streamsize>::max(), '_'))) return false;
    if (!(std::getline(ss, segment, '_'))) return false;
    try { i = std::stoi(segment); } catch (...) { return false; }
    if (!(std::getline(ss, segment, '_'))) return false;
    try { j = std::stoi(segment); } catch (...) { return false; }
    if (j < 1 || j > 4) {
        std::cerr << "Warning: Split index " << j << " for flow " << i << " is outside the expected range [1, 4]. Variable: " << var_name << std::endl;
    }
    if (!(std::getline(ss, segment, '_'))) return false;
    try { u = std::stoi(segment); } catch (...) { return false; }
    if (!(std::getline(ss, segment))) return false;
    try { v = std::stoi(segment); } catch (...) { return false; }
    return true;
}

// Function to reconstruct a path for a given flow split using its edge list
std::pair<Path, int> reconstructPath_multiSplit(int flow_id, int split_id, const std::vector<std::pair<int, int>>& edges) {
    if (edges.empty()) {
        return {{}, 0};
    }
    std::map<int, int> successors;
    std::set<int> edge_starts;
    std::set<int> edge_ends;
    int start_node = -1;

    for (const auto& edge : edges) {
        int u = edge.first;
        int v_node = edge.second;
        if (successors.count(u)) {
            std::cerr << "Error: Flow " << flow_id << ", Split " << split_id << " has multiple outgoing edges from node " << u << ". Path reconstruction failed." << std::endl;
            return {{}, -1};
        }
        successors[u] = v_node;
        edge_starts.insert(u);
        edge_ends.insert(v_node);
    }

    for (int node : edge_starts) {
        if (edge_ends.find(node) == edge_ends.end()) {
            if (start_node != -1) {
                std::cerr << "Error: Flow " << flow_id << ", Split " << split_id << " has multiple potential start nodes (" << start_node << ", " << node << "). Path reconstruction failed." << std::endl;
                return {{}, -1};
            }
            start_node = node;
        }
    }

    if (start_node == -1) {
        if (edges.size() == 1 && edges[0].first == edges[0].second) {
            return {{edges[0].first}, 0};
        } else if (!successors.empty()) {
            std::cerr << "Error: Flow " << flow_id << ", Split " << split_id << ": Cannot determine a unique start node. Check for cycles or incomplete edge data." << std::endl;
        } else {
            std::cerr << "Error: Flow " << flow_id << ", Split " << split_id << ": Cannot determine start node (no edges?). Path reconstruction failed." << std::endl;
        }
        return {{}, -1};
    }

    Path path;
    path.push_back(start_node);
    int current_node = start_node;
    std::set<int> visited_nodes_in_path;
    visited_nodes_in_path.insert(start_node);

    while (successors.count(current_node)) {
        int next_node = successors[current_node];
        path.push_back(next_node);
        if (visited_nodes_in_path.count(next_node)) {
            std::cerr << "Error: Flow " << flow_id << ", Split " << split_id << ": Cycle detected involving node " << next_node << ". Path reconstruction failed." << std::endl;
            return {{}, -1};
        }
        visited_nodes_in_path.insert(next_node);
        current_node = next_node;
        if (path.size() > (edges.size() + 1) + 5 ) {
            std::cerr << "Error: Flow " << flow_id << ", Split " << split_id
                      << ": Path reconstruction seems to be in an unexpected loop. Path size: " << path.size() << ", Edges: " << edges.size() << std::endl;
            return {{}, -1};
        }
    }

    int hops = (path.size() > 0) ? (static_cast<int>(path.size()) - 1) : 0;
    if (hops != static_cast<int>(edges.size()) && !path.empty()) {
        std::cerr << "Warning: Flow " << flow_id << ", Split " << split_id << ": Reconstructed path has " << hops << " hops, but "
                << edges.size() << " edges were provided for this split. Path may be incomplete or contain a non-path structure." << std::endl;
    }
    return {path, hops};
}


// Main parsing function (now takes a specific file path)
multiSplitFlowData parseMILPResults_multiSplit(const std::string& file_to_parse_str) {
    std::ifstream infile(file_to_parse_str);
    if (!infile.is_open()) { throw std::runtime_error("Error: Could not open file: " + file_to_parse_str); }

    multiSplitFlowData data;
    std::map<int, std::map<int, std::vector<std::pair<int, int>>>> flow_split_edges;

    std::string line;
    int line_num = 0;
    while (std::getline(infile, line)) {
        line_num++;
        line.erase(0, line.find_first_not_of(" \t\n\r\f\v"));
        line.erase(line.find_last_not_of(" \t\n\r\f\v") + 1);

        if (line.empty() || line[0] == '#') {
            continue;
        }

        size_t equals_pos = line.find('=');
        if (equals_pos == std::string::npos) {
            std::cerr << "Warning: Malformed line (no '=') at line " << line_num << ": " << line << std::endl;
            continue;
        }

        std::string var_name = line.substr(0, equals_pos);
        std::string value_str = line.substr(equals_pos + 1);

        var_name.erase(var_name.find_last_not_of(" \t") + 1);
        var_name.erase(0, var_name.find_first_not_of(" \t"));
        value_str.erase(0, value_str.find_first_not_of(" \t"));
        value_str.erase(value_str.find_last_not_of(" \t") + 1);

        int i = -1, j = -1, u = -1, v_node = -1;

        try {
            if (var_name == "numThreads") {
                data.numThreads = std::stoi(value_str);
                data.mapping.resize(data.numThreads, -1);
            } else if (var_name.rfind("f_", 0) == 0) {
                if (!parseVarName_multiSplit(var_name, i, j, u, v_node)) {
                    std::cerr << "Warning: Could not parse f_ variable name at line " << line_num << ": " << var_name << std::endl;
                    continue;
                }
                double flow_value = std::stod(value_str);
                if (flow_value < 0) {
                    std::cerr << "Warning: Negative flow value " << flow_value << " for " << var_name << " at line " << line_num << std::endl;
                }
                std::pair<int, int> edge = {u, v_node};
                data.edge_loads[edge] += flow_value;

                if (data.flow_split_weights[i].find(j) == data.flow_split_weights[i].end()) {
                    if (flow_value > 1e-9) {
                       data.flow_split_weights[i][j] = flow_value;
                    }
                } else {
                    if (flow_value > 1e-9 && std::abs(data.flow_split_weights[i][j] - flow_value) > 1e-9) {
                         std::cerr << "Warning: Inconsistent flow_value for flow " << i << ", split " << j
                                   << ". Previous: " << data.flow_split_weights[i][j]
                                   << ", Current: " << flow_value << " on edge (" << u << "," << v_node
                                   << ") at line " << line_num << ". Using first non-trivial value encountered." << std::endl;
                    }
                }
            } else if (var_name.rfind("b_", 0) == 0) {
                 if (!parseVarName_multiSplit(var_name, i, j, u, v_node)) {
                     std::cerr << "Warning: Could not parse b_ variable name at line " << line_num << ": " << var_name << std::endl;
                     continue;
                 }
                double binary_value = std::stod(value_str);
                if (binary_value == 1) {
                    flow_split_edges[i][j].push_back({u, v_node});
                } else if (binary_value != 0) {
                     std::cerr << "Warning: Non-binary value for 'b_' variable at line " << line_num << ": " << line << std::endl;
                }
            } else if (var_name.length() >= 2 && var_name[0] == 'O' && std::isdigit(var_name[1])) {
                try {
                    unsigned int objective_idx = std::stoi(var_name.substr(1));
                    data.objectives[objective_idx] = std::stod(value_str);
                } catch (const std::invalid_argument& ia) {
                     std::cerr << "Warning: Could not parse objective index from " << var_name << " at line " << line_num << ": " << ia.what() << std::endl;
                } catch (const std::out_of_range& oor) {
                     std::cerr << "Warning: Objective index out of range in " << var_name << " at line " << line_num << ": " << oor.what() << std::endl;
                }
            } else if(var_name.length() == 5 && var_name[0] == 'g') {
                try {
                    unsigned int threadId = std::stoi(var_name.substr(2));
                    unsigned int vertexId = std::stoi(var_name.substr(4));
                    if (std::stoi(value_str) == 1)
                        data.mapping[threadId] = vertexId;
                } catch (const std::invalid_argument& ia) {
                    std::cerr << "Warning: Could not parse objective index from " << var_name << " at line " << line_num << ": " << ia.what() << std::endl;
                } catch (const std::out_of_range& oor) {
                    std::cerr << "Warning: Objective index out of range in " << var_name << " at line " << line_num << ": " << oor.what() << std::endl;
                }
            }
            else {
                //  std::cerr << "Warning: Unrecognized line format at line " << line_num << ": " << line << std::endl;
            }
        } catch (const std::invalid_argument& e) {
            std::cerr << "Warning: Invalid number format at line " << line_num << ": " << value_str << " for variable " << var_name << " (" << e.what() << ")" << std::endl;
        } catch (const std::out_of_range& e) {
             std::cerr << "Warning: Number out of range at line " << line_num << ": " << value_str << " for variable " << var_name << " (" << e.what() << ")" << std::endl;
        } catch (...) {
            std::cerr << "Warning: Unknown parsing error at line " << line_num << ": " << line << std::endl;
        }
    }
    infile.close();

    for (const auto& [flow_id, splits_data] : data.flow_split_weights) {
        double current_total_flow_weight = 0.0;
        for (const auto& pair : splits_data) { // the pair is [split_id, weight]
            current_total_flow_weight += pair.second;
        }
        data.total_flow_weights[flow_id] = current_total_flow_weight;
    }

    // std::cout << "DEBUG 3\n";

    data.total_hops = 0;
    for (auto const& [flow_id, splits] : flow_split_edges) {
        for (auto const& [split_id, edges] : splits) {
             if (!edges.empty()) {
                auto [path, hops] = reconstructPath_multiSplit(flow_id, split_id, edges);
                if (hops >= 0) {
                   data.routes[flow_id][split_id] = path;
                   data.total_hops += hops;
                } else {
                     data.routes[flow_id].erase(split_id);
                }
            }
        }
    }
    return data;
}

// Function to write results to a file
void writeResultsToFile(const multiSplitFlowData& data, const std::string& output_filename) {
    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open output file: " << output_filename << std::endl;
        return;
    }

    outfile << std::fixed << std::setprecision(6); // for consistent double formatting

    // --- Metrics Section ---
    outfile << "## Metrics" << std::endl;
    if (data.objectives.empty()) {
        outfile << "(No Metrics found)" << std::endl;
    } else {
        outfile << "MCL = " << data.objectives.at(1) << std::endl;
        outfile << "Total Hop Count = " << data.objectives.at(2) << std::endl;
    }

    // --- Power Section ---
    outfile << "Power Consumption (W) = " << data.total_power << std::endl;

    outfile << std::endl;

    // --- Mapping Section ---
    if (data.mapping.empty() == false) {
        outfile << "## Thread Mapping" << std::endl;
        outfile << "Thread ID: ";
        for (size_t t = 0; t < data.mapping.size(); ++t) {
            outfile << t << " ";
        }
        outfile << "\nVertex ID: ";
        for (size_t t = 0; t < data.mapping.size(); ++t) {
            outfile << data.mapping[t] << " ";
        }
        outfile << "\n\n";
    }

    // --- Routes Section ---
    outfile << "## Routes" << std::endl;
    if (data.routes.empty()) {
        outfile << "(No routes found or reconstructed)" << std::endl;
    } else {
        for (const auto& [flow_id, splits_routes] : data.routes) {
            double total_weight = 0.0;
            if (data.total_flow_weights.count(flow_id)) {
                total_weight = data.total_flow_weights.at(flow_id);
            }
            // outfile << "## Flow " << flow_id << " with total weight = " << total_weight << std::endl;

            if (splits_routes.empty()) {
                 outfile << "(No splits for this flow)" << std::endl;
            } else {
                for (const auto& [split_id, path] : splits_routes) {
                    double split_load = 0.0;
                    if (data.flow_split_weights.count(flow_id) && data.flow_split_weights.at(flow_id).count(split_id)) {
                        split_load = data.flow_split_weights.at(flow_id).at(split_id);
                    }
                    // outfile << "### split " << split_id << " with load = " << split_load << std::endl;

                    if (path.empty()) {
                        outfile << "(Path empty or failed reconstruction for this split)" << std::endl;
                    } else {
                        for (size_t k = 0; k < path.size(); ++k) {
                            outfile << path[k] << (k == path.size() - 1 ? "" : " ");
                        }
                        outfile << std::endl;
                    }
                    outfile << split_load << std::endl;
                }
            }
             outfile << std::endl;
        }
    }
    outfile.close();
    std::cout << "Info: Results written to " << output_filename << std::endl;
}
