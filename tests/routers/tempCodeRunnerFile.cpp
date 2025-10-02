#include <iostream>
// #include "../../headers/topologies/Butterfly.h"
// #include "../../headers/mappers/StaticMapper.h"
// #include "../../headers/utils/CommunicationGraph.h"
#include "../../Application/utils/networks/topologies/Mesh.hpp"
#include "../../Application/Routing/include/Valiant.hpp"
#include "../../Application/Routing/include/DestinationTagRouter.hpp"
#include "../../Application/Routing/include/DimensionOrderRouter.hpp"
#include "../../Application/Routing/include/XorTagRouter.hpp"
#include "../../Application/Routing/include/MinimalOR.hpp"
#include "../../Application/Routing/include/LBOR.hpp"

#include <fstream>

void simulate(My_Router &r, int hopcount, std::vector<int> path, int src, int dst)
{
    try
    {
        DimensionOrderRouter *dor = dynamic_cast<DimensionOrderRouter *>(&r);
        if (dor != nullptr)
        {
            dor->set_xy_first(false);
        }

        path = r.route(src, dst, hopcount, 1);
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
    // Create a 3x3 mesh topology
    int n = 9;
    Mesh mesh(n, n);
    mesh.draw_mesh();
    // mesh.print_nodes_list();
    // mesh.print_links_src_dst();
    // CommunicationGraph graph(n * n);
    // mesh.corrupt_links_between(11,7,0.1);
    vector<std::string> corrupted = mesh.corrupt_links(0.05);
    std::cout << "Corrupted links:\n";
    for (auto x : corrupted)
    {
        std::cout << x << std::endl;
    }

    // Use a StaticMapper to map the communication graph to the topology
    // StaticMapper mapper;
    // std::vector<int> mapping = mapper.map(graph, &mesh);

    // Initialize the Dimension Order Router with the mesh topology
    DimensionOrderRouter dor(&mesh);
    Valiant valiant(&mesh);
    DestinationTagRouter dtr(&mesh);
    MinimalOR minimalor(&mesh);
    LBOR lbor(&mesh);
    XORTagRouter xortag(&mesh);

    // open input file and read src and dst from input.txt
    std::ifstream file;
    file.open("input.txt");

    int src = -1, dst = -1;
    file >> src >> dst;
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
    // std::cout << "***********xortag************\n";
    // simulate(xortag, hopcount, path, src, dst);
    std::cout << "***********dtr************\n";
    simulate(dtr, hopcount, path, src, dst);

    return 0;
}