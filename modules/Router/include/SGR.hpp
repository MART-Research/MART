#include "../utils/milps/orUtils.hpp"
#include "../utils/milps/MARTUtility.hpp"
#include "../utils/milps/MILPSolver.hpp"
#include <regex>

// Forward declaration of the MILP solver function
namespace operations_research {
    void solveLP(
        MILPSolver& milp_solver,
        std::multimap<int, std::pair<int, double>> &flows, 
        std::unordered_map<int, bool> &sources,
        std::unordered_map<int, bool> &destinations,
        std::unordered_map<int, std::vector<int>> &flowNetwork,
        std::unordered_map<int, std::vector<int>> &reverseFlowNetwork,
        int OBJECTIVE_NUM
    );
}

#ifndef SGR_H
#define SGR_H


struct Edge {
    int src;
    int dest;
    double weight;

    int getEndNode() const { return dest; }
    double getWt() const { return weight; }
};

class Edges {
public:
    std::vector<Edge> edgeList;

    bool hasEdge(int node) const {
        return std::any_of(edgeList.begin(), edgeList.end(), [node](const Edge& edge) { return edge.src == node; });
    }

    Edge getLargestWtEdge(int node) const {
        auto it = std::max_element(edgeList.begin(), edgeList.end(), [node](const Edge& a, const Edge& b) {
            return (a.src == node ? a.weight : -1) < (b.src == node ? b.weight : -1);
        });
        return *it;
    }

    void subtractListWt(const std::vector<Edge>& edges, double wt) {
        for (auto& edge : edgeList) {
            for (const auto& e : edges) {
                if (edge.src == e.src && edge.dest == e.dest) {
                    edge.weight -= wt;
                    // remove fully processed edges; we simply nullify the edge (set src and dest to -1) instead of actually removing it which would require resizing the vector O(n)
                    if (edge.weight <= 0) {
                        edge.src = edge.dest = -1;
                    }
                }
            }
        }
    }
};

struct Flow {
    int src;
    std::vector<Edge> edges;

    int getSrcNode() const { return src; }

    const Edges getEdges() const {
        Edges edgesStruct;
        edgesStruct.edgeList = edges;
        return edgesStruct;
    }
};

class PeelPaths {
public:
    std::vector<std::vector<int>> paths;
    std::vector<double> pathWeights;

    void addPeelPath(const std::vector<int>& nodes, double weight) {
        paths.emplace_back(nodes);
        pathWeights.emplace_back(weight);
    }
};

class SGR {
public:
    SGR
    (
        MILPSolver& solver,
        int num_nodes,
        std::unordered_map<int, std::vector<int>> &flowNetwork,
        std::multimap<int, std::pair<int, double>> &communicationGraph,
        int OBJECTIVE_NUM
    ) : 
    milpSolver(solver),
    numNodes(num_nodes),
    flowNetwork(flowNetwork),
    commGraph(communicationGraph),
    OBJECTIVE_NUM(OBJECTIVE_NUM)
    {}

    std::vector<Flow> parseFlows(const std::string& filepath)
    {
        std::unordered_map<int,int> id2src;int id=1;
        for(const auto& e:commGraph)id2src[id++]=e.first;

        std::filesystem::path p(filepath);
        std::ifstream in(p);
        if(!in)throw std::runtime_error("cannot open file");

        std::unordered_map<int,std::vector<Edge>> bucket;
        std::string line;
        while(std::getline(in,line)){
            if(line.empty()||line[0]!='f')continue;
            int fid,u,v;double w;
            if(std::sscanf(line.c_str(),"f_%d_%d_%d = %lf",&fid,&u,&v,&w)==4)
                if(w!=0.0)bucket[fid].push_back({u,v,w});
        }

        std::vector<Flow> flows;
        for(auto& [fid,edges]:bucket)flows.push_back({id2src[fid],std::move(edges)});
        return flows;
    }

    void printFlows(const std::vector<Flow>& flows)
    {
        for (const auto& f : flows) {
            std::cout << "Flow from " << f.src << '\n';
            for (const auto& e : f.edges)
                std::cout << "  " << e.src << " -> " << e.dest
                        << "  w = " << e.weight << '\n';
        }
    }

    void printPeelPath(const PeelPaths &peelpaths) {
        // Print the resulting peel paths
        for (size_t i = 0; i < peelpaths.paths.size(); ++i) {
            std::cout << "Path: ";
            for (size_t j = 0; j < peelpaths.paths[i].size(); ++j) {
                std::cout << peelpaths.paths[i][j] << " ";
            }
            std::cout << "Weight: " << peelpaths.pathWeights[i] << "\n";
        }
    }

    PeelPaths run_sgr() {
        // Assign sources and destinations
        for (const auto& flow : commGraph) {
            sources[flow.first] = true;
            destinations[flow.second.first] = true;
        }

        // Create the reverse flow network
        for (const auto& vertex : flowNetwork) {
            int src = vertex.first;
            const auto& vec = vertex.second;
            for (const auto& dst : vec) {
                reverseFlowNetwork[dst].emplace_back(src);
            }
        }

        operations_research::solveLP(milpSolver, commGraph, sources, destinations, flowNetwork, reverseFlowNetwork, OBJECTIVE_NUM); // produces flows and saves them to a file

        // Call the flow peeling function here
        std::vector<Flow> flows = parseFlows("app/output/milps/sgr_milp_output.txt");

        // printFlows(flows);

        PeelPaths peelpaths;
        greedyFlowPeeling(flows, peelpaths);

        // printPeelPath(peelpaths);

        return peelpaths;
    }

private:
    void greedyFlowPeeling(std::vector<Flow>& flows, PeelPaths& peelpaths) {
        for (auto& flow : flows) {
            int src_node = flow.getSrcNode();
            Edges edges = flow.getEdges();

            while (edges.hasEdge(src_node)) {
                double wt = INF;
                int node = src_node;
                std::vector<int> peelpath_nodes;
                std::vector<Edge> list_edges;

                do {
                    Edge edge = edges.getLargestWtEdge(node);
                    if (edge.getWt() < wt) {
                        wt = edge.getWt();
                    }
                    peelpath_nodes.push_back(node);
                    list_edges.push_back(edge);
                    node = edge.getEndNode();
                } while (edges.hasEdge(node));

                peelpath_nodes.push_back(node);
                edges.subtractListWt(list_edges, wt);
                peelpaths.addPeelPath(peelpath_nodes, wt);
            }
        }
    }

    MILPSolver& milpSolver;
    int numNodes;
    std::unordered_map<int, std::vector<int>> flowNetwork;
    std::multimap<int, std::pair<int, double>> commGraph;  // The flows, each flow is represented as {src, {dest, demand}}
    int OBJECTIVE_NUM;

    std::unordered_map<int, bool> sources, destinations;
    std::unordered_map<int, std::vector<int>> reverseFlowNetwork;
};

#endif



namespace operations_research {

void solveLP(
    MILPSolver& milp_solver,
    std::multimap<int, std::pair<int, double>> &flows, 
    std::unordered_map<int, bool> &sources, 
    std::unordered_map<int, bool> &destinations, 
    std::unordered_map<int, std::vector<int>> &flowNetwork, 
    std::unordered_map<int, std::vector<int>> &reverseFlowNetwork, 
    int OBJECTIVE_NUM
    )  
    {
    // Retrieve the solver
    MPSolver* solver = milp_solver.get();
    

    // Define the Variables
    int M = (int)flows.size();
    std::map<std::tuple<int, int, int>, MPVariable*> f; // fi(u,v) : load carried for the ith flow on the edge (u,v)

    int i = 1;
    for (const auto& flow : flows) {
        const int si = flow.first, ti = flow.second.first;
        const double li = flow.second.second;
        std::vector<std::pair<int, int>> TraversibleEdges;
        std::unordered_map<int, bool> isVisited;
        std::unordered_map<int, bool> leadsToTi;
        // generateTraversibleEdges(ti, si, flowNetwork, isVisited, leadsToTi, TraversibleEdges);
        generateTraversibleEdgesBFS(si, ti, flowNetwork, leadsToTi, TraversibleEdges);
        
        // for (size_t kk = 0; kk < TraversibleEdges.size(); ++kk) std::cout <<"{" << TraversibleEdges[kk].first << ", " << TraversibleEdges[kk].second << "}, "; cout << endl;

        for (const auto& edge : TraversibleEdges) {
            const int &u = edge.first, &v = edge.second;
            std::string flowVarName = "f_" + std::to_string(i) + "_" + std::to_string(u) + "_" + std::to_string(v);
            f[{i, u, v}] = solver->MakeNumVar(0.0, li, flowVarName);
        }
        ++i;
    }

    // Define the constraints
    // C1

    i = 1;
    for (const auto& flow: flows) {
        int si = flow.first, di = flow.second.first;
        const double li = flow.second.second;
        MPConstraint *const flow_conservation_constraint_src = solver->MakeRowConstraint(li, li, "C11_" + std::to_string(i) + "_" + std::to_string(si));
        MPConstraint *const flow_conservation_constraint_dst = solver->MakeRowConstraint(li, li, "C12_" + std::to_string(i) + "_" + std::to_string(di));
        for (const auto& v : flowNetwork.at(si))
            flow_conservation_constraint_src->SetCoefficient(f[{i, si, v}], 1);
        for (const auto& u : reverseFlowNetwork.at(di))
            flow_conservation_constraint_dst->SetCoefficient(f[{i, u, di}], 1);
        ++i;
    }

    // Note: Can a naming contradiction occur between C11 and C12 constraints if si = di? If so, how to handle it? Remember to think about it.
    // If so, it could be handled by, for example, adding the letter 's' or 'd' to the constraint name to differentiate between the two.

    // C2
    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first, di = flow.second.first;
        const double li = flow.second.second;
        for (const auto& u : reverseFlowNetwork.at(si)) {
            std::string constraintName = "C21_" + std::to_string(i) + "_" + std::to_string(u) + "_" + std::to_string(si);
            MPConstraint *const C21_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);
            C21_constraint->SetCoefficient(f[{i, u, si}], 1);
        }

        for (const auto& v : flowNetwork.at(di)) {
            std::string constraintName = "C22_" + std::to_string(i) + "_" + std::to_string(di) + "_" + std::to_string(v);
            MPConstraint *const C22_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);
            C22_constraint->SetCoefficient(f[{i, di, v}], 1);
        }
        ++i;
    }

    // C3
    i = 1;
    for (const auto& flow: flows) {
        int si = flow.first, di = flow.second.first;
        for (const auto& node : flowNetwork) {
            int v = node.first;
            if (v == si || v == di) continue;   // v = si or v = ti
            std::string constraintName = "C3_" + std::to_string(i) + "_" + std::to_string(v);
            MPConstraint *const flow_conservation_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);
            const auto& vec = node.second;
            for (const auto& u : vec)
                flow_conservation_constraint->SetCoefficient(f[{i, v, u}], 1);
            const auto& rev_vec = reverseFlowNetwork.at(v);
            for (const auto& u: rev_vec)
                flow_conservation_constraint->SetCoefficient(f[{i, u, v}], -1);
        }
        ++i;
    }

    // Possible Objectives / Cost Functions
    // 1] O1 : minimize the number of hops (if minimal routing is needed)
    
    // O1
    operations_research::MPVariable* const S = solver->MakeNumVar(0.0, solver->infinity(), "O1");
    std::string constraintName = "O1C";
    MPConstraint* const O1C_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);
    O1C_constraint->SetCoefficient(S, -1);
    for (const auto& node : flowNetwork) {
        const int& u = node.first;
        for (const auto& v : node.second) {
            int i = 1;
            for (const auto& flow : flows) {
                O1C_constraint->SetCoefficient(f[{i, u, v}], 1);
                ++i;
            }
        }
    }

    // O2
    // MPObjective* const O2 = solver->MutableObjective();
    operations_research::MPVariable* const Z = solver->MakeNumVar(0.0, solver->infinity(), "O2");
    for (const auto& node : flowNetwork) {
        const int& u = node.first;
        for (const auto & v : node.second) {
            std::string constraintName = "O2C_" + std::to_string(u) + "_" + std::to_string(v);
            MPConstraint* const O2C_constraint = solver->MakeRowConstraint(-solver->infinity(), 0.0, constraintName);
            O2C_constraint->SetCoefficient(Z, -1);
            int i = 1;
            for (const auto& flow : flows) {
                O2C_constraint->SetCoefficient(f[{i, u, v}], 1);
                ++i;
            }    
        }    
    }

    // Set objective function coefficients depending on the main sub-objective
    // Here we are implementing a simple weighing, but a more rigorous coefficient selection should be souhgt later
    // Search about techniques of coefficient selection when working with multi-objective linear optimization problems

    int S_Coefficient, Z_Coefficient;
    if (OBJECTIVE_NUM == 1) {
        S_Coefficient = 1e6, Z_Coefficient = 1.0;
    } 
    else {
        S_Coefficient = 1.0, Z_Coefficient = 1e6;
    }
    operations_research::MPObjective* const objective = solver->MutableObjective();
    objective->SetCoefficient(S, S_Coefficient);   // higher priority objective
    objective->SetCoefficient(Z, Z_Coefficient);   // lower priority objective
    objective->SetMinimization();

    solver->EnableOutput(); // enable solver's own log output
    // Solve the problem
    const MPSolver::ResultStatus result_status = solver->Solve();

    std::stringstream solverLog;
    milp_solver.getSolutionLog(result_status, objective, solverLog);
    
    writeLogToFile(solverLog, "app/output/milps/sgr_milp_output.txt");
    WriteVariablesToFile(*solver, "app/output/milps/sgr_milp_output.txt", "append");
}

}  // namespace operations_research
