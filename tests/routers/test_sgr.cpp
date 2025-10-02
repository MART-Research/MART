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
#include "../../modules/utils/milps/MARTUtility.hpp"
#include "../../modules/Router/include/SGR.hpp"

int main(int argc, char* argv[]) {
    // Initialize Google flags first
    InitGoogle(argv[0], &argc, &argv, true);
    absl::SetFlag(&FLAGS_stderrthreshold, 0);

    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <flowNetworkFile> <flowsFile> objective_num\n";
        std::cerr << "Current directory: " << std::filesystem::current_path() << "\n";
        return EXIT_FAILURE;
    }

    // Read flow network and flows from files
    std::cout << "Reading flow network from: " << argv[1] << "\n";
    auto flowNetwork = readFlowNetwork(argv[1]);
    
    std::cout << "Reading flows from: " << argv[2] << "\n";
    auto flows = readFlows_singledest_double(argv[2]);

    unsigned int objectiveNum = std::stoi(argv[3]);

    // Debug print file contents
    printFlowNetwork(flowNetwork);

    std::cout << "\nFlows Contents:\n";
    for (const auto& pair: flows) { std::cout << "src = " << pair.first << ", dst = " << pair.second.first << ", weight = " << pair.second.second << "\n"; }

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

    std::cout << "\nRunning SGR with " << numNodes << " nodes...\n";
    SGR sgr(numNodes, flowNetwork, flows, objectiveNum);
    PeelPaths peelpaths = sgr.run_sgr();

    return EXIT_SUCCESS;
}