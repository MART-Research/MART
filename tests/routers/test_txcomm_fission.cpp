#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <utility>
#include <algorithm>
#include <cmath>
#include <sstream>
#include "../../modules/Router/TransCom.hpp"


std::unordered_map<int, std::vector<int>> readFlowNetwork(const std::string& filename) {
    std::unordered_map<int, std::vector<int>> network;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open flow network file: " << filename << "\n";
        return network;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int src;
        if (!(iss >> src)) continue;  // Skip malformed lines
        
        int dst;
        while (iss >> dst) {
            network[src].push_back(dst);
        }
    }
    return network;
}

std::multimap<int, std::pair<int, int>> readFlows(const std::string& filename) {
    std::multimap<int, std::pair<int, int>> flows;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open flows file: " << filename << "\n";
        return flows;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int weight, src, dst;
        if (!(iss >> weight >> src >> dst)) continue;  // Skip malformed lines
        flows.insert({src, {dst, weight}});
    }
    return flows;
}

int main(int argc, char* argv[]) {
    // Initialize Google flags first
    InitGoogle(argv[0], &argc, &argv, true);
    absl::SetFlag(&FLAGS_stderrthreshold, 0);

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <flowNetworkFile> <flowsFile>\n";
        std::cerr << "Current directory: " << std::filesystem::current_path() << "\n";
        return EXIT_FAILURE;
    }

    // Read files with full path verification
    std::cout << "Reading flow network from: " << argv[1] << "\n";
    auto flowNetwork = readFlowNetwork(argv[1]);
    
    std::cout << "Reading flows from: " << argv[2] << "\n";
    auto flows = readFlows(argv[2]);

    // Debug print file contents
    std::cout << "\nFlow Network Contents:\n";
    for (const auto& [node, neighbors] : flowNetwork) {
        std::cout << node << ": ";
        for (int neighbor : neighbors) {
            std::cout << neighbor << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\nFlows Contents:\n";
    for (const auto& pair: flows) {
        std::cout << "src = " << pair.first << ", dst = " << pair.second.first << ", weight = " << pair.second.second << "\n";
    }

    // Calculate numNodes
    int numNodes = 0;
    for (const auto& [node, neighbors] : flowNetwork) {
        numNodes = std::max(numNodes, node);
        for (int neighbor : neighbors) {
            numNodes = std::max(numNodes, neighbor);
        }
    }

    if (flowNetwork.empty() || flows.empty()) {
        std::cerr << "Error: No data loaded from input files\n";
        return EXIT_FAILURE;
    }

    std::cout << "\nRunning TRANSCOM (Fission Only) with " << numNodes << " nodes...\n";
    TRANSCOM transcom(flowNetwork, flows);
    transcom.run_transcom_fission_only();

    return EXIT_SUCCESS;
}
