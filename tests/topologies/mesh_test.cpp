#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <string>
#include <climits>
#include <cerrno>
#include "../../modules/utils/Networks/Topologies/Mesh.hpp"
#include "../../modules/Router/include/MinimalOblivious.hpp"
#include "../../modules/Router/include/XorTag.hpp"
#include "../../modules/Router/include/DestinationTag.hpp"
#include "../../modules/Router/include/DimensionOrder.hpp"
#include "../../modules/Router/include/LoadBalancedOblivious.hpp"
#include "../../modules/Router/include/Valiant.hpp"

#define ROUTING_TESTS 5
#define DEFAULT_DIM 4
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




void simulate(Router &r, int hopcount, std::vector<int> path, int src, int dst)
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
    int n = 5;
    Mesh mesh(n, n);
    DimensionOrder dor(&mesh);
    Valiant valiant(&mesh);
    DestinationTag dtr(&mesh);
    MinimalOblivious minimalor(&mesh);
    LoadBalancedOblivious lbor(&mesh);
    XORTag xortag(&mesh);

    // open input file and read src and dst from input.txt
    std::ifstream file;
    file.open("input.txt");
    if (!file.is_open())
    {
        std::cerr << "Error: cannot open ‘input.txt’\n";
        return 1;
    }
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < ROUTING_TESTS; i++)
    {
        int src = -1, dst = -1;
        // file >> src >> dst;
        //or generate random src and dst
        mesh.generate_src_dst(src, dst);
        std::cout << "----------Routing from Node " << src << " to Node " << dst << ":-----------\n";
        std::vector<int> path;
        int hopcount = 0;
        int vc = 1;
        double flow = 0.0;
        std::cout << "*********lbor**********\n";
        simulate(lbor, hopcount, path, src, dst);
        
        std::cout << "**********dor**********\n";
        simulate(dor, hopcount, path, src, dst);
        
        std::cout << "***********valiant**********\n";
        simulate(valiant, hopcount, path, src, dst);
        
        std::cout << "***********minimalor**********\n";
        simulate(minimalor, hopcount, path, src, dst);
        
        std::cout << "***********dtr************\n";
        simulate(dtr, hopcount, path, src, dst);
        std::cout << "---------------------------------------------\n";
    }
    std::cout<<"Mesh tests ended successfully.\n";
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Execution time: " << elapsed.count() << " seconds\n";
    return 0;
}