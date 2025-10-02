#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include "../../modules/utils/Networks/Topology.hpp"
#include "../../modules/utils/Networks/Topologies/Dragonfly.hpp"
#include "../../modules/utils/Networks/Topologies/Butterfly.hpp"
#include "../../modules/utils/Networks/Topologies/Hypercube.hpp"
#include "../../modules/utils/Networks/Topologies/Mesh.hpp"
#include "../../modules/utils/Networks/Topologies/Ring.hpp"
#include "../../modules/utils/Networks/Topologies/Torus.hpp"
#include "../../modules/utils/Networks/Topologies/Tree.hpp"
#include "../../modules/Router/include/MinimalOblivious.hpp"
#include "../../modules/Router/include/XorTag.hpp"
#include "../../modules/Router/include/DestinationTag.hpp"
#include "../../modules/Router/include/DimensionOrder.hpp"
#include "../../modules/Router/include/LoadBalancedOblivious.hpp"
#include "../../modules/Router/include/Valiant.hpp"

using namespace std;

static inline string trim(const string &s)
{
    auto b = s.find_first_not_of(" \t\r\n");
    if (b == string::npos)
        return "";
    auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

static vector<string> split(const string &s, char delim)
{
    vector<string> out;
    istringstream ss(s);
    string token;
    while (getline(ss, token, delim))
        out.push_back(token);
    return out;
}

// ——— Dragonfly simulation ———
void simulate_dragonfly(Router &r,
                        Dragonfly &dragon,
                        int hopcount,
                        vector<int> path,
                        int src,
                        int dst)
{
    try
    {
        path = r.route(src, dst, hopcount, 1);
        for (int node : path)
        {
            const auto &routers = dragon.get_routers();
            if (node < (int)routers.size() && routers[node] != nullptr)
                cout << routers[node]->get_router_name() << " -> ";
            else
                cout << "Node " << node << " -> ";
        }
        cout << "Destination Reached!\n"
             << "Hopcount: " << hopcount << "\n";
    }
    catch (const out_of_range &e)
    {
        cerr << "Error: " << e.what() << "\n";
        return;
    }
    cout << "simulation ended\n\n";
}

void simulate_butterfly(Router &r,
                        int hopcount,
                        vector<int> path,
                        int src,
                        int dst)
{
    try
    {
        path = r.route(src, dst, hopcount, 1);
        for (int node : path)
        {
            cout << "Node " << node << " -> ";
        }
        cout << "Destination Reached!\n"
             << "Hopcount: " << hopcount << "\n\n";
    }
    catch (const out_of_range &e)
    {
        cerr << "Error: " << e.what() << "\n";
        return;
    }
}

void simulate_mesh(Router &r, int hopcount, std::vector<int> path, int src, int dst)
{
    try
    {
        DimensionOrder *dor = dynamic_cast<DimensionOrder *>(&r);
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

void simulate_hypercube(Router &r, int hopcount, std::vector<int> path, int src, int dst)
{
    try
    {
        DimensionOrder *dor = dynamic_cast<DimensionOrder *>(&r);
        if (dor != nullptr)
        {
            dor->set_xy_first(false);
            // path = r.route(src, dst, hopcount, 1);
        }
        // if(dor==nullptr)
        path = r.route(src, dst, hopcount, 1);
        // std::cout<<"after routing\n";
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
void simulate_tree(Router &r, int hopcount, std::vector<int> path, int src, int dst)
{
    try
    {
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
    // parse config into map<string,string>
    unordered_map<string, string> cfg;
    ifstream ifs("config");
    if (!ifs)
    {
        cerr << "Error: cannot open ‘config’\n";
        return 1;
    }
    string line;
    while (getline(ifs, line))
    {
        line = trim(line);
        if (line.empty() || line[0] == '#')
            continue;
        auto eq = line.find('=');
        if (eq == string::npos)
            continue;
        string key = trim(line.substr(0, eq));
        string val = trim(line.substr(eq + 1));
        if (!val.empty() && val.back() == ';')
            val.pop_back();
        cfg[key] = trim(val);
    }

    // build topology  remember its kind
    string tn = cfg["topology"];
    transform(tn.begin(), tn.end(), tn.begin(), ::tolower);

    unique_ptr<Dragonfly> dragon;
    unique_ptr<Butterfly> butterfly;
    unique_ptr<Mesh> mesh;
    unique_ptr<Ring> ring;
    // unique_ptr<Torus_2D> torus2d;
    // unique_ptr<Torus_3D> torus3d;
    unique_ptr<Torus> torus;
    unique_ptr<Tree> tree;
    unique_ptr<Hypercube> hypercube;
    int hopcount = stoi(cfg["hopcount"]);
    int vcs = stoi(cfg["num_vcs"]);
    float faulty_percentage = stof(cfg["faulty_links"]);
    if (faulty_percentage < 0.0f || faulty_percentage > 1.0f)
    {
        cerr << "Error: faulty_links must be between 0.0 and 1.0\n";
        return 1;
    }
    if (tn == "dragonfly")
    {
        int G = stoi(cfg["num_groups"]);
        int Rg = stoi(cfg["routers_per_group"]);
        int Nr = stoi(cfg["nodes_per_router"]);
        int Gl = stoi(cfg["global_links_per_router"]);
        dragon = make_unique<Dragonfly>(G, Rg, Nr, Gl, vcs);
        dragon->corrupt_links(faulty_percentage);
    }
    else if (tn == "butterfly")
    {
        int k = stoi(cfg["k"]);
        if (k & (k - 1))
        {
            cerr << "Error: butterfly 'k' must be a power of two\n";
            return 1;
        }
        butterfly = make_unique<Butterfly>(k);
    }
    else if (tn == "hypercube")
    {
        int dims = stoi(cfg["num_dimensions"]);
        hypercube = make_unique<Hypercube>(dims, vcs);
        hypercube->corrupt_links(faulty_percentage);
    }
    else if (tn == "mesh")
    {
        int rows = stoi(cfg["num_rows"]);
        int cols = stoi(cfg["num_cols"]);
        mesh = make_unique<Mesh>(rows, cols, vcs);
        mesh->corrupt_links(faulty_percentage);
    }
    else if (tn == "ring")
    {
        int nodes = stoi(cfg["nodes"]);
        ring = make_unique<Ring>(nodes, vcs);
        ring->corrupt_links(faulty_percentage);
    }
    else if (tn == "torus")
    {
        int rows = stoi(cfg["rows"]);
        int cols = stoi(cfg["cols"]);
        int depth = stoi(cfg["depth"]);
        torus = make_unique<Torus>(rows, cols, depth, vcs);
    }
    else if (tn == "tree")
    {
        int levels = stoi(cfg["levels"]);
        int branches = stoi(cfg["branches"]);
        tree = make_unique<Tree>(levels, branches, vcs);
        tree->corrupt_links(faulty_percentage);
    }
    else
    {
        cerr << "Unknown topology: " << cfg["topology"] << "\n";
        return 1;
    }

    // instantiate the routers 
    vector<unique_ptr<Router>> routers;
    Topology *base = nullptr;
    if (dragon)
    {
        base = dragon.get();
    }
    else if (butterfly)
    {
        base = butterfly.get();
    }
    else if (mesh)
    {
        base = mesh.get();
    }
    else if (ring)
    {
        base = ring.get();
    }
    else if (torus)
    {
        base = torus.get();
    }

    else if (tree)
    {
        base = tree.get();
    }
    else if (hypercube)
    {
        base = hypercube.get();
    }
    for (auto &raw : split(cfg["routing_algorithms"], ','))
    {
        string n = trim(raw);
        if (n == "minimalor" || n == "min")
            routers.emplace_back(make_unique<MinimalOblivious>(base));
        else if (n == "xortag" || n == "xor")
            routers.emplace_back(make_unique<XORTag>(base));
        else if (n == "destinationtag" || n == "desttag" || n == "dtr")
            routers.emplace_back(make_unique<DestinationTag>(base));
        else if (n == "dimensionorder" || n == "dor")
            routers.emplace_back(make_unique<DimensionOrder>(base));
        else if (n == "lbor")
            routers.emplace_back(make_unique<LoadBalancedOblivious>(base));
        else if (n == "valiant")
            routers.emplace_back(make_unique<Valiant>(base));
        else
            cerr << "Warning: unknown router \"" << n << "\"\n";
    }

    vector<pair<int, int>> routes;
    for (auto &part : split(cfg["routes"], ','))
    {
        auto a = part.find("->");
        if (a == string::npos)
            continue;
        int src = stoi(trim(part.substr(0, a)));
        int dst = stoi(trim(part.substr(a + 2)));
        routes.emplace_back(src, dst);
    }

    for (auto &p : routes)
    {
        cout << "=== Routing from " << p.first
             << " to " << p.second << " ===\n";
        for (auto &r : routers)
        {
            if (dragon)
            {
                simulate_dragonfly(*r, *dragon, hopcount, {}, p.first, p.second);
            }
            else if (butterfly)
            {
                simulate_butterfly(*r, hopcount, {}, p.first, p.second);
            }
            else if (mesh)
            {
                simulate_mesh(*r, hopcount, {}, p.first, p.second);
            }
            else if (ring)
            {
                // simulate_ring(*r, hopcount, {}, p.first, p.second);
            }
            else if (torus)
            {
                // simulate_torus2d(*r, hopcount, {}, p.first, p.second);
            }
       
            else if (tree)
            {
                simulate_tree(*r, hopcount, {}, p.first, p.second);
            }
            else if (hypercube)
            {
                simulate_hypercube(*r, hopcount, {}, p.first, p.second);
            }
        }
    }

    return 0;
}
