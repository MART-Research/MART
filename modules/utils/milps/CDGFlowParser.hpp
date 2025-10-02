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
struct CDGFlowData {
    std::map<int, Path> routes;
    std::map<int, double> flowDemand;
    std::map<std::pair<int, int>, double> edge_loads;
    long long total_hops = 0;
    double mcl = 0;
    std::map<int, double> objectives;
    double total_power;
    vector<int> mapping;
};

bool parseVarName(const std::string& var_name, int& i, int& edge1Src, int& edge1Dst, int& edge2Src, int& edge2Dst) {
    std::stringstream ss(var_name);
    std::string segment;

    // Skip the first character ('f' or 'b') and the first '_'
    if (!(ss.ignore(std::numeric_limits<std::streamsize>::max(), '_'))) return false;

    if (!(std::getline(ss, segment, '_'))) return false;
    try { i = std::stoi(segment); } catch (...) { return false; }

    if (!(std::getline(ss, segment, '_'))) return false;
    try { edge1Src = std::stoi(segment); } catch (...) { return false; }

    if (!(std::getline(ss, segment, '_'))) return false;
    try { edge1Dst = std::stoi(segment); } catch (...) { return false; }

    if (!(std::getline(ss, segment, '_'))) return false;
    try { edge2Src = std::stoi(segment); } catch (...) { return false; }

    if (!(std::getline(ss, segment))) return false;
    try { edge2Dst = std::stoi(segment); } catch (...) { return false; }

    return true;
}

std::pair<Path, int> reconstructPathCDG(int flow_id, const std::vector<std::pair<int, int>>& edges) {
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
        std::cerr << "Warning: Flow " << flow_id
                  << ": Reconstructed path has " << hops << " hops, but "
                  << edges.size() << " edges were provided. Path may be incomplete." << std::endl;
    }
    return {path, hops};
}


CDGFlowData parseCDGMILPResults(const std::string& file_to_parse_str) {
    std::ifstream infile(file_to_parse_str);
    if (!infile.is_open()) { throw std::runtime_error("Error: Could not open file: " + file_to_parse_str); }

    CDGFlowData data;
    std::map<int, std::vector<std::pair<int, int>>> flow_edges;

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

        int i = -1, edge1Src = -1, edge1Dst = -1, edge2Src = -1, edge2Dst = -1;

        try {
            if (var_name.rfind("f_", 0) == 0) {
                if (!parseVarName(var_name, i, edge1Src, edge1Dst, edge2Src, edge2Dst)) {
                    std::cerr << "Warning: Could not parse f_ variable name at line " << line_num << ": " << var_name << std::endl;
                    continue;
                }
                double flow_value = std::stod(value_str);
                if (flow_value < 0) {
                    std::cerr << "Warning: Negative flow value " << flow_value << " for " << var_name << " at line " << line_num << std::endl;
                } else if (flow_value > 0) {
                    data.flowDemand[i] = flow_value;
                }
                std::pair<int, int> edge1 = {edge1Src, edge1Dst};
                std::pair<int, int> edge2 = {edge2Src, edge2Dst};
                if (edge1Src != -1) data.edge_loads[edge1] += flow_value;
                if (edge2Dst != -1) data.edge_loads[edge2] += flow_value;
            } else if (var_name.rfind("b_", 0) == 0) {
                 if (!parseVarName(var_name, i, edge1Src, edge1Dst, edge2Src, edge2Dst)) {
                     std::cerr << "Warning: Could not parse b_ variable name at line " << line_num << ": " << var_name << std::endl;
                     continue;
                 }
                double binary_value = std::stod(value_str);
                if (binary_value == 1) {
                    if (edge1Src == -1) {
                        flow_edges[i].push_back({edge1Dst, edge2Dst});
                    } else if (edge2Dst == -1) {
                        
                    } else if (edge1Src != -1 && edge2Dst != -1) {
                        flow_edges[i].push_back({edge1Dst, edge2Dst});
                    }
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

    data.total_hops = 0;
    for (auto const& [flow_id, edges] : flow_edges) {
         if (!edges.empty()) {
            auto [path, hops] = reconstructPathCDG(flow_id, edges);
            if (hops >= 0) {
               data.routes[flow_id] = path;
               data.total_hops += hops;
            }
        }
    }
    // std::cout << "DEBUG 3\n";
    std::pair<int, int> max_edge;
    for (const auto& [edge, load] : data.edge_loads) {
        if (load > data.mcl) {
            data.mcl = load;
            max_edge = edge;
        }   
    }
    // std::cout << "(" << max_edge.first << ", " << max_edge.second << ") -> weight = " << data.mcl << std::endl;
    data.mcl /= 2;
    return data;
}

// Function to write results to a file
void writeResultsToFile(const CDGFlowData& data, const std::string& output_filename) {
    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open output file: " << output_filename << std::endl;
        return;
    }

    outfile << std::fixed << std::setprecision(6);

    // --- Metrics Section ---
    outfile << "## Metrics" << std::endl;
    if (data.objectives.empty()) {
        outfile << "(No Metrics found)" << std::endl;
    } else {
        for (const auto& [obj_idx, obj_val] : data.objectives) {
            if (obj_idx == 1) {
                outfile << "Total Throughput = ";
            } else if (obj_idx == 2) {
                outfile << "Max min-fairness = ";
            } else {
                outfile << "MCL = ";
            }
            outfile << obj_val << std::endl;
        }
    }
    outfile << "Total Hop Count = " << data.total_hops << std::endl;

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
        for (const auto& [flow_id, route] : data.routes) {
            // outfile << "## Flow " << flow_id << " with total weight = " << data.flowDemand.at(flow_id) << std::endl;
            for (const auto& node: route) {
                outfile << node << " ";
            }
            outfile << std::endl;
            outfile << data.flowDemand.at(flow_id) << std::endl;
        }
    }
    outfile.close();
    std::cout << "Info: Results written to " << output_filename << std::endl;
}
