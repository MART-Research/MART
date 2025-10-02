#include <iostream>
#include <fstream>
#include "../../modules/utils/networks/topologies/Tree.hpp"
#include "../../modules/Router/include/MinimalOblivious.hpp"
#include "../../modules/Router/include/XorTag.hpp"
#include "../../modules/Router/include/DestinationTag.hpp"
#include "../../modules/Router/include/DimensionOrder.hpp"
#include "../../modules/Router/include/LoadBalancedOblivious.hpp"
#include "../../modules/Router/include/Valiant.hpp"
#define ROUTING_TESTS 5
void simulate(Router &r, int hopcount, std::vector<int> path, int src, int dst, int vc = 1, double flow = 0.0)

{
    try
    {
        path = r.route(src, dst, hopcount, 1);
        if (path.empty() || (path.size() == 2 && path[0] == -1 && path[1] == -1))
        {
            std::cout << "FAILED: No active links found for routing from Node " << src << " to Node " << dst << ".\n";
            return;
        }
        for (int node : path)
        {
            std::cout << "Node " << node << " -> ";
        }
        std::cout << "Destination Reached!\n";

        std::cout << std::endl;
        std::cout << "Hopcount: " << hopcount << std::endl;
    }
    catch (const std::out_of_range &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return;
    }
}

int main(int argc, char *argv[])
{

    auto start = std::chrono::high_resolution_clock::now();
    try
    {
        int n, k;
        // try reading from atoi, if not assign 2 and 3 respectively
        if (argc > 2)
        {
            n = atoi(argv[1]);
            k = atoi(argv[2]);
        }
        else
        {
            n = 4;
            k = 5;
        }
        Tree tree(n, k);
        DestinationTag dtr(&tree);
        std::ifstream file;
        file.open("input.txt");
        int src = -1, dst = -1;
        for (int i = 0; i < ROUTING_TESTS; i++)
        {
            // file >> src >> dst;
            tree.generate_src_dst(src, dst);
            std::cout << "Routing from Node " << src << " to Node " << dst << ":\n";
            std::vector<int> path;
            int hopcount = 0;
            int vc = 1;
            double flow = 0.0;
            std::cout << "***********dtr************\n";
            simulate(dtr, hopcount, path, src, dst, vc, flow);
            std::cout << "---------------------------------------------------------------\n";
        }
        std::cout << "Tree tests ended successfully!\n";
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
g++ tree_test.cpp -o tree_test
.\tree_test 2 3
*/