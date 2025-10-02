#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <algorithm>

// all your topologies:
#include "../../modules/utils/Networks/Topology.hpp"
#include "../../modules/utils/Networks/Topologies/Dragonfly.hpp"
#include "../../modules/utils/Networks/Topologies/Butterfly.hpp"
#include "../../modules/utils/Networks/Topologies/Hypercube.hpp"
#include "../../modules/utils/Networks/Topologies/Mesh.hpp"
#include "../../modules/utils/Networks/Topologies/Ring.hpp"
#include "../../modules/utils/Networks/Topologies/Torus.hpp"
#include "../../modules/utils/Networks/Topologies/Tree.hpp"
// #include "../../modules/Router/Router.hpp"
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
    string tok;
    while (getline(ss, tok, delim))
        out.push_back(tok);
    return out;
}

int main()
{
    unordered_map<string, string> cfg;
    ifstream ifs("config_mor");
    if (!ifs)
    {
        cerr << "ERROR: cannot open config_mor\n";
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

    // 2) build the chosen topology
    string tn = cfg["topology"];
    transform(tn.begin(), tn.end(), tn.begin(), ::tolower);

    unique_ptr<Dragonfly> dragon;
    unique_ptr<Butterfly> butterfly;
    unique_ptr<Hypercube> hypercube;
    unique_ptr<Mesh> mesh;
    unique_ptr<Ring> ring;
    unique_ptr<Torus> torus;
    unique_ptr<Tree> tree;

    // common params
    int vcs = stoi(cfg["num_vcs"]);
    int hopcount = stoi(cfg["hopcount"]);
    double flow = stod(cfg["flow"]);
    float faulty = 0.0f;
    if (cfg.count("faulty_links"))
    {
        faulty = stof(cfg["faulty_links"]);
        if (faulty < 0.0f || faulty > 1.0f)
        {
            cerr << "ERROR: faulty_links must be between 0 and 1\n";
            return 1;
        }
    }

    if (tn == "dragonfly")
    {
        int G = stoi(cfg["num_groups"]);
        int Rg = stoi(cfg["routers_per_group"]);
        int Nr = stoi(cfg["nodes_per_router"]);
        int Gl = stoi(cfg["global_links_per_router"]);
        dragon = make_unique<Dragonfly>(G, Rg, Nr, Gl, vcs);
        if (faulty > 0)
            dragon->corrupt_links(faulty);
    }
    else if (tn == "butterfly")
    {
        int k = stoi(cfg["k"]);
        if (k & (k - 1))
        {
            cerr << "ERROR: butterfly k must be power of two\n";
            return 1;
        }
        butterfly = make_unique<Butterfly>(k);
        if (faulty > 0)
        {
            butterfly->corrupt_links(faulty);
        }
    }
    else if (tn == "hypercube")
    {
        int dims = stoi(cfg["num_dimensions"]);
        hypercube = make_unique<Hypercube>(dims, vcs);
        if (faulty > 0)
            hypercube->corrupt_links(faulty);
    }
    else if (tn == "mesh")
    {
        int r = stoi(cfg["num_rows"]);
        int c = stoi(cfg["num_cols"]);
        mesh = make_unique<Mesh>(r, c, vcs);
        if (faulty > 0)
            mesh->corrupt_links(faulty);
    }
    else if (tn == "ring")
    {
        int n = stoi(cfg["nodes"]);
        ring = make_unique<Ring>(n, vcs);
        if (faulty > 0)
            ring->corrupt_links(faulty);
    }
    else if (tn == "torus")
    {
        int r = stoi(cfg["torus_rows"]);
        int c = stoi(cfg["torus_cols"]);
        int d = stoi(cfg["torus_depth"]);
        torus = make_unique<Torus>(r, c, d, vcs, faulty);
    }

    else if (tn == "tree")
    {
        int levels = stoi(cfg["levels"]);
        int branches = stoi(cfg["branches"]);
        tree = make_unique<Tree>(levels, branches, vcs);
        if (faulty > 0)
            tree->corrupt_links(faulty);
    }
    else
    {
        cerr << "ERROR: unknown topology " << cfg["topology"] << "\n";
        return 1;
    }

    // pick base pointer
    Topology *topo = nullptr;
    if (dragon)
        topo = dragon.get();
    else if (butterfly)
        topo = butterfly.get();
    else if (hypercube)
        topo = hypercube.get();
    else if (mesh)
        topo = mesh.get();
    else if (ring)
        topo = ring.get();
    else if (torus)
        topo = torus.get();
    else if (tree)
        topo = tree.get();

    assert(topo && "Toplogy not instantiated");

    MinimalOblivious dor(topo);
    int num_tests = stoi(cfg["num_tests"]);
    for (int i = 0; i < num_tests; ++i)
    {
        // parse route_i
        string rk = "route_" + to_string(i);
        auto arr = split(cfg[rk], '-'); // ["0", ">7"]
        int src = stoi(trim(arr[0]));
        int dst = stoi(trim(arr[1].substr(1))); // skip '>'

        // parse expected_i
        string ek = "expected_" + to_string(i);
        auto parts = split(cfg[ek], ','); // e.g. ["0","1","5","7"]
        vector<int> expected;
        for (auto &p : parts)
            expected.push_back(stoi(trim(p)));

        vector<int> path = dor.route(src, dst, hopcount, vcs, flow);

        // cout << "size of expected path: " << expected.size() << "\n";
        // cout << "size of actual   path: " << path.size() << "\n";

        // print expected
        cout << "expected path [test_" << i << " (" << src << "->" << dst << ")]: ";
        for (size_t j = 0; j < expected.size(); ++j)
        {
            cout << expected[j] << (j + 1 < expected.size() ? " -> " : "");
        }
        cout << "\n";

        // print actual
        cout << "actual   path [test_" << i << " (" << src << "->" << dst << ")]: ";
        for (size_t j = 0; j < path.size(); ++j)
        {
            cout << path[j] << (j + 1 < path.size() ? " -> " : "");
        }
        cout << "\n";

        // now assert (you’ll see both lines above even if this fails)
        assert(path.size() == expected.size());
        for (size_t j = 0; j < path.size(); ++j)
        {
            assert(path[j] == expected[j]);
        }

        cout << "[PASS] test_" << i << " (" << src << "->" << dst << ")\n\n";
    }

    cout << "All MinimalOblivious tests passed!\n";
    return 0;
}
