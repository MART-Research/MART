#include <iostream>
#include <fstream>
#include "../../modules/utils/Networks/Topologies/Ring.hpp"
#include "../../modules/Router/include/MinimalOblivious.hpp"
#include "../../modules/Router/include/XorTag.hpp"
#include "../../modules/Router/include/DestinationTag.hpp"
#include "../../modules/Router/include/DimensionOrder.hpp"
#include "../../modules/Router/include/LoadBalancedOblivious.hpp"
#include "../../modules/Router/include/Valiant.hpp"
#define ROUTING_TESTS 5
int numSuccess = 0;

void simulate(Router &r, int hopcount, std::vector<int> path, int src, int dst, int vc = 1, double flow = 0.0)

{
    try
    {
        DimensionOrder *dor = dynamic_cast<DimensionOrder *>(&r);
        if (dor != nullptr)
        {
            dor->set_xy_first(false);
        }

        path = r.route(src, dst, hopcount, vc, flow);
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
        // numSuccess++;
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
        int n;
        if (argc > 1)
        {
            n = atoi(argv[1]);
        }
        else
        {
            n = 30;
        }
        Ring ring(n);
        DimensionOrder dor(&ring);
        Valiant valiant(&ring);
        DestinationTag dtr(&ring);
        MinimalOblivious minimalor(&ring);
        LoadBalancedOblivious lbor(&ring);
        XORTag xortag(&ring);

        std::ifstream file;
        file.open("input.txt");
        if (!file.is_open())
        {
            std::cerr << "Failed to open input file: input.txt\n";
            return 1;
        }
        int src = -1, dst = -1;
        int hopcount = 0;
        for (int i = 0; i < ROUTING_TESTS; i++)
        {
            file >> src >> dst;
            // or generate random src and dst
            //  ring.generate_src_dst(src, dst);
            std::cout << "Routing from Node " << src << " to Node " << dst << ":\n";
            std::vector<int> path;
            int vc = 1;
            double flow = 0.0;
            std::cout << "***********lbor************\n";
            simulate(lbor, hopcount, path, src, dst, vc, flow);

            std::cout << "***********dor************\n";
            simulate(dor, hopcount, path, src, dst, vc, flow);

            std::cout << "***********valiant************\n";
            simulate(valiant, hopcount, path, src, dst, vc, flow);

            std::cout << "***********minimalor************\n";
            simulate(minimalor, hopcount, path, src, dst, vc, flow);

            std::cout << "***********dtr************\n";
            simulate(dtr, hopcount, path, src, dst, vc, flow);
            std::cout << "----------------------------------------\n";
        }
        std::cout << " Ring tests ended successfully!\n";
        numSuccess = 0;
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

g++ ring_test.cpp -o ring_test
.\ring_test

*/