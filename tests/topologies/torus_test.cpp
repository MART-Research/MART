#include <iostream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include "../../modules/utils/Networks/Topologies/Torus.hpp"
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
        DimensionOrder *dor = dynamic_cast<DimensionOrder *>(&r);
        if (dor != nullptr)
        {
            dor->set_xy_first(false);
        }

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
        std::cout << "Hopcount: " << hopcount << std::endl;
    }
    catch (const std::out_of_range &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return;
    }
}

int main()
{

    auto start = std::chrono::high_resolution_clock::now();
    try
    {
        int n = 3;
        int vcs = 1;
        double faulty_links = 0.01; // 1% of links will be corrupted
        Torus torus(n, n, n, vcs, faulty_links);
    
        DimensionOrder dor(&torus);
        Valiant valiant(&torus);
        DestinationTag dtr(&torus);
        MinimalOblivious minimalor(&torus);
        LoadBalancedOblivious lbor(&torus);
        XORTag xortag(&torus);
        std::ifstream file;
        file.open("input.txt");
        if (!file.is_open())
        {
            std::cerr << "Failed to open input file: input.txt\n";
            return 1;
        }

        int src = -1, dst = -1;
        for (int i = 0; i < ROUTING_TESTS; i++)
        {
            // file >> src >> dst;
            // std::cout << src << " " << dst << std::endl;
            torus.generate_src_dst(src, dst);
            std::cout << "Routing from Node " << src << " to Node " << dst << ":\n";
            std::vector<int> path;
            int hopcount = 0;
            std::cout << "***********lbor************\n";
            simulate(lbor, hopcount, path, src, dst);

            std::cout << "***********dor************\n";
            simulate(dor, hopcount, path, src, dst);

            std::cout << "***********valiant************\n";
            simulate(valiant, hopcount, path, src, dst);

            std::cout << "***********minimalor************\n";
            simulate(minimalor, hopcount, path, src, dst);

            // simulate(xortag, hopcount, path, src, dst);
            // needs significant edits to work with torus
            // std::cout << "***********dtr************\n";
            // simulate(dtr, hopcount, path, src, dst
            std::cout << "---------------------------------------------------------------\n";
        }
        std::cout << "Torus tests ended successfully!\n";
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