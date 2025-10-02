#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <climits>
#include <cerrno>
#include "../../modules/utils/Networks/Topologies/Butterfly.hpp"
#include "../../modules/Router/include/MinimalOblivious.hpp"
#include "../../modules/Router/include/XorTag.hpp"
#include "../../modules/Router/include/DestinationTag.hpp"

#define ROUTING_TESTS 5
#define DEFAULT_K 8
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

void simulate(Router &router, int hopcount, std::vector<int> path, int src, int dst, int vc = 1, double flow = 0.0)
{
    try
    {
        path = router.route(src, dst, hopcount, vc, flow);
        if (path.empty() || (path.size() == 2 && path[0] == -1 && path[1] == -1))
        {
            std::cout << "FAILED: No active links found for routing from Node " << src << " to Node " << dst << ".\n";
            return;
        }
        std::cout << "Path: ";
        for (size_t i = 0; i < path.size(); ++i)
        {
            std::cout << path[i];
            if (i < path.size() - 1)
                std::cout << " -> ";
        }
        std::cout << "\nHopcount: " << hopcount << "\n";
        numSuccess++;
    }
    catch (const std::out_of_range &e)
    {
        std::cerr << "Routing failed: " << e.what() << std::endl;
    }
}

int main(int argc, char *argv[])
{
    auto start = std::chrono::high_resolution_clock::now();
    try
    {
        int k;
        k = parseIntOrDefault(argc > 1 ? argv[1] : nullptr, 8);
        int num_vcs = parseIntOrDefault(argc > 2 ? argv[2] : nullptr, 1);
        double faulty_links = 0.00;
        Butterfly butterfly(k, num_vcs, faulty_links);
        butterfly.print_links_src_dst();
        butterfly.print_topology();
        butterfly.print_nodes_list();
        DestinationTag dtr(&butterfly);
        MinimalOblivious minimalor(&butterfly);
        XORTag xortag(&butterfly);
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
            file >> src >> dst;
            // //or generate random src and dst
            //  butterfly.generate_src_dst(src, dst);
            std::cout << "--------Routing from Node " << src << " to Node " << dst << ":------\n";
            std::vector<int> path;
            int hopcount = 0;
            double flow = 0.0;

            std::cout << "***********minimalor************\n";
            simulate(minimalor, hopcount, path, src, dst, num_vcs, flow);

            // std::cout << "***********xortag************\n";
            // simulate(xortag, hopcount, path, src, dst, num_vcs, flow);

            std::cout << "***********dtr************\n";
            simulate(dtr, hopcount, path, src, dst, num_vcs, flow);

            std::cout << "---------------------------------------------------------------\n";
        }
        std::cout << "Butterfly tests ended successfully!\n";
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
g++ butterfly_test.cpp -o butterfly_test
 .\butterfly_test 32

*/