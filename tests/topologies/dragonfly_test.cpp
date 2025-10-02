#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include "../../modules/utils/Networks/Topologies/Dragonfly.hpp"
#include "../../modules/Router/include/MinimalOblivious.hpp"
#include "../../modules/Router/include/XorTag.hpp"
#include "../../modules/Router/include/DestinationTag.hpp"
#include "../../modules/Router/include/DimensionOrder.hpp"
#include "../../modules/Router/include/LoadBalancedOblivious.hpp"
#include "../../modules/Router/include/Valiant.hpp"
#define ROUTING_TESTS 5
int numSuccess = 0;
void simulate(Router &r, Dragonfly &dragon, int hopcount, std::vector<int> path, int src, int dst)
{
    try
    {
        path = r.route(src, dst, hopcount, 1);
        if (path.empty() || (path.size() == 2 && path[0] == -1 && path[1] == -1))
        {
            cout << "FAILED: No active links found for routing from Node " << src << " to Node " << dst << ".\n";
            return;
        }
        for (int node : path)
        {
            if (node < dragon.get_routers().size() && dragon.get_routers()[node] != nullptr)
                std::cout << dragon.get_routers()[node]->get_router_name() << " -> ";
            else
                std::cout << "Node " << node << " -> ";
        }
        std::cout << "Destination Reached!\n";
        std::cout << "Hopcount: " << hopcount << std::endl;
        
    }
    catch (const std::out_of_range &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    numSuccess++;
}

int main()
{
    auto start = std::chrono::high_resolution_clock::now();
    try
    {
        int num_groups = 5;
        int routers_per_group = 3;
        int nodes_per_router = 4;
        int global_links_per_router = 2;
        int num_vcs = 1;
        double faulty_links = 0.04; // we assume 4% faulty links, a random 4% of the links will be corrupted
        Dragonfly dragon(num_groups, routers_per_group, nodes_per_router, global_links_per_router, num_vcs, faulty_links);
        // dragon.print_neighbors();
        DestinationTag dtr(&dragon);
        MinimalOblivious minimalor(&dragon);
        XORTag xor_tag(&dragon);
        std::ifstream file;
        /*
             // in case we want to corrupt specific links, we can use the following
              dragon.corrupt_links_between(0, 1);
              dragon.corrupt_links_between(1, 2);
             //or to corrupt links randomly, we can use the following
              vector<std::string> corrupted = dragon.corrupt_links(0.1);
              std::cout << "Corrupted links are:\n";
              for (auto x : corrupted)
              {
                  std::cout << x << std::endl;
              }
      */
        file.open("input.txt");
        if (!file.is_open())
        {
            std::cerr << "Failed to open input file.\n";
            return 1;
        }
        int src = -1, dst = -1;
        for (int i = 0; i < ROUTING_TESTS; i++)
        {
            // Read source and destination from the file
            // file >> src >> dst;

            // or generate random source and destinations
            dragon.generate_src_dst(src, dst);
            std::cout << "-----------Routing from Node " << src << " to Node " << dst << "-----------:\n";
            std::vector<int> path;
            int hopcount = 0;
            std::cout << "***********minimalor************\n";
            simulate(minimalor, dragon, hopcount, path, src, dst);
            std::cout << "***********dtr************\n";
            simulate(dtr, dragon, hopcount, path, src, dst);
            std::cout << "---------------------------------------------------------------\n";
        }
        std::cout << "Dragonfly tests ended successfully!\n";
    }
    catch (const std::exception &e)
    {
        std::cout << "Caught an exception: " << e.what() << std::endl;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;
    return 0;
}

/*
g++ dragonfly_test.cpp -o dragonfly_test
.\dragonfly_test
*/


//  if (numSuccess == ROUTING_TESTS)
//     {
//         std::cout << "[PASS] All tests passed for Destination Tag Routing.\n";
//     }
//     else
//     {
//         std::cout << "[PASS] only " << numSuccess << "/" << ROUTING_TESTS << " tests passed for Destination Tag Routing.\n";
//     }
//     numSuccess = 0;
//     std::cout << "----------------------------------------\n";