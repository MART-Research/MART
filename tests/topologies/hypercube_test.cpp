#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include "../../modules/utils/Networks/Topologies/Hypercube.hpp"
#include "../../modules/Router/include/MinimalOblivious.hpp"
#include "../../modules/Router/include/XorTag.hpp"
#include "../../modules/Router/include/DestinationTag.hpp"
#include "../../modules/Router/include/DimensionOrder.hpp"
#include "../../modules/Router/include/LoadBalancedOblivious.hpp"
#include "../../modules/Router/include/Valiant.hpp"
#define ROUTING_TESTS 5

int numSuccess = 0;

int parseIntOrDefault(const char *str, int defaultValue)
{
    if (!str)
        return defaultValue;

    char *end = nullptr;
    errno = 0;
    long val = std::strtol(str, &end, 10);

    if (errno == 0 && *end == '\0' && val >= INT_MIN && val <= INT_MAX)
        return static_cast<int>(val);

    std::cerr << "Invalid input \"" << str << "\". Using default value: " << defaultValue << std::endl;
    return defaultValue;
}

void simulate(Router &r, int hopcount, std::vector<int> path, int src, int dst, int vc = 1, double flow = 0.0)
{
    try
    {
        DimensionOrder *dor = dynamic_cast<DimensionOrder *>(&r);
        if (dor != nullptr)
        {
            dor->set_xy_first(false);
        }
        // std::cout<<"control before routing\n";
        path = r.route(src, dst, hopcount, vc, flow);
        if (path.empty() || (path.size() == 2 && path[0] == -1 && path[1] == -1))
        {
            std::cout << "FAILED: No active links found for routing from Node " << src << " to Node " << dst << ".\n";
            return;
        }
        std::cout << "after routing\n";
        for (int node : path)
        {
            std::cout << "Node " << node << " -> ";
        }
        std::cout << "Destination Reached!\n";
        std::cout << "Hopcount: " << hopcount << std::endl;
        numSuccess++;
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
        int n = parseIntOrDefault(argc > 1 ? argv[1] : nullptr, 9); //default to a hypercube of dimension 9, 2^9 = 512 nodes
        int vcs = parseIntOrDefault(argc > 2 ? argv[2] : nullptr, 1);
        double faulty_links = 0.03;
        Hypercube cube(n, vcs, faulty_links);
        DimensionOrder dor(&cube);
        Valiant valiant(&cube);
        DestinationTag dtr(&cube);
        MinimalOblivious minimalor(&cube);
        LoadBalancedOblivious lbor(&cube);
        XORTag xortag(&cube);

        std::ifstream file("input.txt");
        if (!file.is_open())
        {
            std::cerr << "Failed to open input file: input.txt\n";
            return 1;
        }

        int src = -1, dst = -1;
        for (int i = 0; i < ROUTING_TESTS; i++)
        {
            file >> src >> dst;
            // or generate random src and dst
            //  hypercube.generate_src_dst(src, dst);
            std::cout << "--------Routing from Node " << src << " to Node " << dst << ":------\n";

            std::vector<int> path;
            int hopcount = 0;

            // std::cout << "***********lbor************\n";
            // simulate(lbor, hopcount, path, src, dst);

            std::cout << "***********dor************\n";
            simulate(dor, hopcount, path, src, dst);

            std::cout << "***********valiant************\n";
            simulate(valiant, hopcount, path, src, dst);

            std::cout << "***********minimalor************\n";
            simulate(minimalor, hopcount, path, src, dst);

            std::cout << "***********dtr************\n";
            simulate(dtr, hopcount, path, src, dst);

            std::cout << "***********xortag************\n";
            simulate(xortag, hopcount, path, src, dst);

            std::cout << "---------------------------------------------------------------\n";
        }

        std::cout << "Hypercube tests ended successfully!\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Caught exception: " << e.what() << std::endl;
        return 1;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;

    return 0;
}
/*

g++ hypercube_test.cpp -o hypercube_test
.\hypercube_test

*/
/*
Compile:
    g++ hypercube_test.cpp -o hypercube_test
Run:
    ./hypercube_test
*/
