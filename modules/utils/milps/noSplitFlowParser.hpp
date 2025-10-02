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

struct noSplitFlowData {
    std::map<int, Path> routes;                         // routes[flow_id] = path (vector of node ids)
    std::map<int, double> flow_weights;                 // flow_weights[flow_id] = weight of that specific flow
    std::map<std::pair<int, int>, double> edge_loads;   // edge_loads[{u, v}] = total flow amount on edge (u, v)
    long long total_hops = 0;                           // total_hops = sum of hops across all paths
    std::map<int, double> objectives;                   // vector of objectives (e.g., objectives[1] for O1, objectives[2] for O2)
    double total_power = 0.0;
    vector<int> mapping;
};

// Helper function to parse the variable name like "f_i_u_v" or "b_i_u_v" (No 'j')
// Returns true on success, false on failure.
bool parseVarNameNoSplit(const std::string& var_name, int& i, int& u, int& v) {
    std::stringstream ss(var_name);
    std::string segment;

    // Skip the first character ('f' or 'b') and the first '_'
    if (!(ss.ignore(std::numeric_limits<std::streamsize>::max(), '_'))) return false;

    // Parse i
    if (!(std::getline(ss, segment, '_'))) return false;
    try { i = std::stoi(segment); } catch (...) { return false; }

    // Parse u
    if (!(std::getline(ss, segment, '_'))) return false;
    try { u = std::stoi(segment); } catch (...) { return false; }

    // Parse v (last segment), it has no more underscores
    if (!(std::getline(ss, segment))) return false;
    try { v = std::stoi(segment); } catch (...) { return false; }

    return true;
}

// Function to reconstruct a path for a given flow using its edge list (No 'split_id')
std::pair<Path, int> reconstructPath(int flow_id, const std::vector<std::pair<int, int>>& edges) {
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
            std::cerr << "Error: Flow " << flow_id << " has multiple outgoing edges from node " << u << ". Path reconstruction failed." << std::endl;
            return {{}, -1};
        }
        successors[u] = v_node;
        edge_starts.insert(u);
        edge_ends.insert(v_node);
    }

    for (int node : edge_starts) {
        if (edge_ends.find(node) == edge_ends.end()) {
            if (start_node != -1) {
                std::cerr << "Error: Flow " << flow_id << " has multiple potential start nodes (" << start_node << ", " << node << "). Path reconstruction failed." << std::endl;
                return {{}, -1};
            }
            start_node = node;
        }
    }

    if (start_node == -1) {
        if (edges.size() == 1 && edges[0].first == edges[0].second) {
            return {{edges[0].first}, 0};
        } else if (!successors.empty()) {
            std::cerr << "Error: Flow " << flow_id << ": Cannot determine a unique start node. Check for cycles or incomplete edge data." << std::endl;
        } else {
            std::cerr << "Error: Flow " << flow_id << ": Cannot determine start node (no edges?). Path reconstruction failed." << std::endl;
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
            std::cerr << "Error: Flow " << flow_id << ": Cycle detected involving node " << next_node << ". Path reconstruction failed." << std::endl;
            return {{}, -1};
        }
        visited_nodes_in_path.insert(next_node);
        current_node = next_node;
    }

    int hops = (path.size() > 0) ? (static_cast<int>(path.size()) - 1) : 0;
    if (hops != static_cast<int>(edges.size()) && !path.empty()) {
        std::cerr << "Warning: Flow " << flow_id << ": Reconstructed path has " << hops << " hops, but " << edges.size() << " edges were provided. Path may be incomplete." << std::endl;
    }
    return {path, hops};
}


// Main parscing function (No Splits version)
noSplitFlowData parseMILPResultsNoSplits(const std::string& file_to_parse_str) {
    std::ifstream infile(file_to_parse_str);
    if (!infile.is_open()) throw std::runtime_error("Error: Could not open file: " + file_to_parse_str);

    noSplitFlowData data;
    std::map<int, std::vector<std::pair<int, int>>> flow_edges;

    std::string line;
    int line_num = 0;
    while (std::getline(infile, line)) {
        line_num++;
        line.erase(0, line.find_first_not_of(" \t\n\r\f\v"));
        line.erase(line.find_last_not_of(" \t\n\r\f\v") + 1);

        if (line.empty() || line[0] == '#') continue;

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

        try {
            if (var_name.length() >= 2 && var_name[0] == 'O' && std::isdigit(var_name[1])) {
                try {
                    int objective_idx = std::stoi(var_name.substr(1));
                    data.objectives[objective_idx] = std::stod(value_str);
                } catch (const std::invalid_argument& ia) {
                     std::cerr << "Warning: Could not parse objective index from " << var_name << " at line " << line_num << ": " << ia.what() << std::endl;
                } catch (const std::out_of_range& oor) {
                     std::cerr << "Warning: Objective index out of range in " << var_name << " at line " << line_num << ": " << oor.what() << std::endl;
                }
            } else {
                // Parse total power or other single variables
                if (var_name == "total_power") {
                     try {
                         data.total_power = std::stod(value_str);
                     } catch(...) {
                         std::cerr << "Warning: Could not parse total_power value at line " << line_num << ": " << value_str << std::endl;
                     }
                } else {
                    // std::cerr << "Warning: Unrecognized line format at line " << line_num << ": " << line << std::endl;
                }
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

    // std::cout << "DEBUG 3\n";

    // Path Reconstruction Phase
    data.total_hops = 0;
    for (auto const& [flow_id, edges] : flow_edges) {
         if (!edges.empty()) {
            auto [path, hops] = reconstructPath(flow_id, edges);
            if (hops >= 0) {
               data.routes[flow_id] = path;
               data.total_hops += hops;
            }
        }
    }
    return data;
}

// Function to write results to a file
void writeResultsToFileNoSplits(const noSplitFlowData& data, const std::string& output_filename) {
    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open output file: " << output_filename << std::endl;
        return;
    }

    outfile << std::fixed << std::setprecision(6);

    // --- Metrics Section ---
    outfile << "## Metrics" << std::endl;
    if (data.objectives.empty()) {
        outfile << "(No objectives found)" << std::endl;
    } else {
        outfile << "MCL = " << data.objectives.at(2) << std::endl;
        outfile << "Total Hop Count = " << data.objectives.at(1) << std::endl;
    }
    // --- Power Section
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

    // --- Routes Section (No Splits) ---
    outfile << "## Routes" << std::endl;
    if (data.routes.empty()) {
        outfile << "(No routes found or reconstructed)" << std::endl;
    } else {
        for (const auto& [flow_id, path] : data.routes) {
            double weight = 0.0;
            if (data.flow_weights.count(flow_id)) {
                weight = data.flow_weights.at(flow_id);
            }

            // outfile << "## Route " << flow_id << " with weight = " << weight << std::endl;

            if (path.empty()) {
                outfile << "(Path empty or failed reconstruction)" << std::endl;
            } else {
                for (size_t k = 0; k < path.size(); ++k) {
                    outfile << path[k] << (k == path.size() - 1 ? "" : " ");
                }
                outfile << std::endl;
            }
            outfile << weight << std::endl;
        }
    }
    outfile.close();
    std::cout << "Info: Results written to " << output_filename << std::endl;
}
