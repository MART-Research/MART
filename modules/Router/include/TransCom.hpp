#include "../utils/milps/orUtils.hpp"
#include "../utils/milps/MARTUtility.hpp"
#include "../utils/milps/MILPSolver.hpp"

// Forward declaration of the MILP solver function
namespace operations_research {
    void solve_transcom_fission_only_MILP(
        MILPSolver& milp_solver,
        const std::multimap<int, std::pair<int, double>> &flows, 
        const std::unordered_map<int, std::vector<int>> &flowNetwork,
        const std::unordered_map<int, std::vector<int>> &reverseFlowNetwork
    );

    void solve_transcom_fission_fusion_MILP(
        MILPSolver& milp_solver,
        std::unordered_map<int, std::pair<std::unordered_set<int>, double>> &flows, // {src, {Di, demand}}
        std::unordered_map<int, std::vector<int>> &flowNetwork, 
        std::unordered_map<int, std::vector<int>> &reverseFlowNetwork,
        std::unordered_map<std::pair<int, int>, int, pair_hash> &min_hops
    );
    
    // helper that prints the exact call-site whenever a map::at throws
    template <class Map, class Key>
    auto safe_at(Map& m, const Key& k, const char* file, int line)
            -> decltype(m.at(k))
    {
        try {
            return m.at(k);
        } catch (const std::out_of_range&) {
            std::cerr << "std::out_of_range at " << file << ':' << line << "  (missing key = " << k << ")\n";
            throw;
        }
    }
    #define SAFE_AT(map,key)  safe_at(map,key,__FILE__,__LINE__)
}

#ifndef TRANSCOM_H
#define TRANSCOM_H

class TRANSCOM {
public:
    TRANSCOM // Constructor for fission only version
    (
        MILPSolver& solver,
        const std::unordered_map<int, std::vector<int>> &flowNetwork,
        const std::multimap<int, std::pair<int, double>> &communicationGraph
    ) : 
    milpSolver(solver),
    flowNetwork(flowNetwork),
    commGraph(communicationGraph)
    {}

    TRANSCOM // Constructor for fission + fusion version
    (
        MILPSolver& solver,
        std::unordered_map<int, std::vector<int>> &flowNetwork,
        std::unordered_map<int, std::pair<std::unordered_set<int>, double>> communicationGraph, 
        int num_nodes
    ) :
    milpSolver(solver),
    numNodes(num_nodes),
    flowNetwork(flowNetwork),
    commGraphFusion(communicationGraph)
    {}

    void run_transcom_fission_only() {
        // Create the reverse flow network
        for (const auto& vertex : flowNetwork) {
            int src = vertex.first;
            const auto& vec = vertex.second;
            for (const auto& dst : vec) {
                reverseFlowNetwork[dst].emplace_back(src);
            }
        }
        operations_research::solve_transcom_fission_only_MILP(milpSolver, commGraph, flowNetwork, reverseFlowNetwork); // produces flows and saves them to a file
    }

    void run_transcom_fission_fusion() {
        // Create the reverse flow network
        for (const auto& vertex : flowNetwork) {
            int src = vertex.first;
            const auto& vec = vertex.second;
            for (const auto& dst : vec) {
                reverseFlowNetwork[dst].emplace_back(src);
            }
        }

        // Find hop values for the minimal routes between the flows
        std::vector<std::pair<int, int>> flowPairs;
        for (const auto& flow : commGraphFusion) {
            int source = flow.first;  // Source node
            const auto& destinationsSet = flow.second.first;  // Destination set (Di)
        
            // Add all source-destination pairs to flowPairs
            for (const auto& dest : destinationsSet) {
                flowPairs.push_back({source, dest});
            }
        }
        EdgeList edgeList = extractEdgeList(flowNetwork);
        std::unordered_map<std::pair<int, int>, int, pair_hash> hop = shortestPathsForPairs(edgeList, flowPairs, numNodes);
        printHopMap(hop);
        for (auto& entry : hop) {
            const auto& key = entry.first; // The pair (source, destination)
            auto& value = entry.second;    // The shortest path value
            if (value == INF) { // No path exists between source and destination
                std::cout << "NO solution: No path exists between the flow nodes: " << key.first << " -> " << key.second << "\n";
                return;
            }
        }

        operations_research::solve_transcom_fission_fusion_MILP(milpSolver, commGraphFusion, flowNetwork, reverseFlowNetwork, hop); // produces flows and saves them to a file
    }

private:
    MILPSolver& milpSolver;
    int numNodes;
    std::unordered_map<int, std::vector<int>> flowNetwork;
    std::multimap<int, std::pair<int, double>> commGraph;  // The flows, each flow is represented as {src, {dest, demand}}
    std::unordered_map<int, std::pair<std::unordered_set<int>, double>> commGraphFusion;  // The flows, each flow is represented as {src, {Di, demand}}, where Di = set of destinations
    std::unordered_map<int, std::vector<int>> reverseFlowNetwork;
};
#endif

namespace operations_research {

void solve_transcom_fission_only_MILP(
    MILPSolver& milp_solver,
    const std::multimap<int, std::pair<int, double>> &flows, 
    const std::unordered_map<int, std::vector<int>> &flowNetwork, 
    const std::unordered_map<int, std::vector<int>> &reverseFlowNetwork
    )  
    {
    // Retrieve the solver
    MPSolver* solver = milp_solver.get();

    // Degree of fission is always 4, as specified in the paper
    int K = 4;

    // Define the Variables
    int M = (int)flows.size();
    std::map<std::tuple<int, int, int, int>, MPVariable*> f; // fi,j(u,v) : flow variable for split j of flow i on edge (u,v)
    std::map<std::tuple<int, int, int, int>, MPVariable*> b; // bi,j(u,v) : boolean variable for split j of flow i on edge (u,v)

    int i = 1;
    for (const auto& flow : flows) {
        const int si = flow.first, ti = flow.second.first;
        const double li = flow.second.second;
        
        std::vector<std::pair<int, int>> TraversibleEdges;
        std::unordered_map<int, bool> isVisited;
        std::unordered_map<int, bool> leadsToTi;
    
        // Check if there's a valid path
        if (!generateTraversibleEdges(ti, si, flowNetwork, isVisited, leadsToTi, TraversibleEdges)) {
            std::cerr << "Warning: No path found for flow " << i << " (si=" << si << ", ti=" << ti << ")\n";
            ++i;
            continue;
        }

        // for (size_t kk = 0; kk < TraversibleEdges.size(); ++kk) {
        //     std::cout <<"{" << TraversibleEdges[kk].first << ", " << TraversibleEdges[kk].second << "}, ";
        // }
        // cout << endl;
    
        for (const auto& edge : TraversibleEdges) {
            const int &u = edge.first, &v = edge.second;
            for (int j = 1; j <= K; j++) {
                const std::string baseName = "_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                f[{i, j, u, v}] = solver->MakeNumVar(0.0, li, "f" + baseName);
                b[{i, j, u, v}] = solver->MakeBoolVar("b" + baseName);
            }
        }
        ++i;
    }

    // Define the constraints

    // ---------------------------------- C1 -----------------------------------------------------------------------------------------
    // end-to-end channel load conservation for each flow; i.e., the sum of the loads on the flow-splits leaving the source
    // (and a similar sum reaching the destination) qij is equal to the original (unsplit) channel flow, which is li
    // This constraint, in conjunction with (C3-2), ensure that at most one link from/to the source has
    // the same load as qij with other fij(u,v) values being zero
    
    // i = 1;
    // for (const auto& flow: flows) {
    //     int si = flow.first, di = flow.second.first;
    //     double li = flow.second.second;
    //     MPConstraint *const flow_conservation_constraint_src = solver->MakeRowConstraint(li, li, "C11_" + std::to_string(i) + "_" + std::to_string(si));
    //     MPConstraint *const flow_conservation_constraint_dst = solver->MakeRowConstraint(li, li, "C12_" + std::to_string(i) + "_" + std::to_string(di));
    //     for (int j = 1; j <= K; ++j) {
    //         for (const auto& v : flowNetwork[si])
    //             flow_conservation_constraint_src->SetCoefficient(f[{i, j, si, v}], 1);
    //         for (const auto& u : reverseFlowNetwork[di])
    //             flow_conservation_constraint_dst->SetCoefficient(f[{i, j, u, di}], 1);
    //     }
    //     ++i;
    // }
    // Note: can a naming contradiction occur between C11 and C12 constraints if si = di? If so, how to handle it? Remember to think about it.
    // If so, it could be handled by, for example, adding the letter 's' or 'd' to the constraint name to differentiate between the two.

    // IMPORTANT NOTE: 
    // The above formualtion is the one in the paper, I think it is CORRECT< more brief and results in a lower number of variables and constraints
    // However, for now I am using this one to comply with the Thesis, it involves defining a new variable qij for each flow i and split j
    i = 1;
    std::map<std::tuple<int, int>, MPVariable*> q; // qi,j : the flow on the j-th split on the i-th multicast tree
    for (const auto& flow : flows) {
        const double li = flow.second.second;
        for (int j = 1; j <= K; j++) {
            std::string flowVarName = "q_" + std::to_string(i) + "_" + std::to_string(j);
            q[{i, j}] = solver->MakeNumVar(0.0, li, flowVarName);
        }
        ++i;
    }

    i = 1;
    for (const auto& flow: flows) {
        int si = flow.first, di = flow.second.first;
        double li = flow.second.second;
        MPConstraint *const flow_conservation_constraint_src = solver->MakeRowConstraint(0.0, 0.0, "C11_" + std::to_string(i) + "_" + std::to_string(si));
        MPConstraint *const flow_conservation_constraint_dst = solver->MakeRowConstraint(0.0, 0.0, "C12_" + std::to_string(i) + "_" + std::to_string(di));
        MPConstraint *const flow_conservation_constraint_total = solver->MakeRowConstraint(li, li, "C13_" + std::to_string(i) + "_" + std::to_string(si) + "_" + std::to_string(di));
        for (int j = 1; j <= K; ++j) {
            flow_conservation_constraint_src->SetCoefficient(q[{i, j}], -1);
            if (flowNetwork.find(si) != flowNetwork.end()) {
                for (const auto& v : flowNetwork.at(si)) flow_conservation_constraint_src->SetCoefficient(f[{i, j, si, v}], 1);
            }

            flow_conservation_constraint_dst->SetCoefficient(q[{i, j}], -1);
            if (reverseFlowNetwork.find(di) != reverseFlowNetwork.end()) {
                for (const auto& u : reverseFlowNetwork.at(di)) flow_conservation_constraint_dst->SetCoefficient(f[{i, j, u, di}], 1);
            }
            
            flow_conservation_constraint_total->SetCoefficient(q[{i, j}], 1);
        }
        ++i;
    }

    // -------------------------------------------------------------------------------------------------------------------------------

    // ---------------------------------- C2 -----------------------------------------------------------------------------------------
    // hop-by-hop flow conservation at intermediate nodes : the sum of incoming flows is equal to the sum of outgoing flows
    i = 1;
    for (const auto& flow: flows) {
        int si = flow.first, di = flow.second.first;        
        for (int j = 1; j <= K; j++) {
            for (const auto& node : flowNetwork) {
                int v = node.first;
                if (v == si || v == di) continue;
                MPConstraint *const flow_conservation_constraint = solver->MakeRowConstraint(0.0, 0.0, "C2_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v));
                
                const auto& vec = node.second;
                for (const auto& u : vec) flow_conservation_constraint->SetCoefficient(f[{i, j, v, u}], 1);

                if (reverseFlowNetwork.find(v) != reverseFlowNetwork.end()) {
                    const auto& rev_vec = reverseFlowNetwork.at(v);
                    for (const auto& u: rev_vec) flow_conservation_constraint->SetCoefficient(f[{i, j, u, v}], -1);
                }
            }
        }
        ++i;
    }

    // -------------------------------------------------------------------------------------------------------------------------------

    // ---------------------------------- C3 -----------------------------------------------------------------------------------------
    // C3-1: Avoiding negative flow values by ensuring that no flow-split exceeds the unsplit flow
    
    i = 1;
    for (const auto& flow: flows) {
        int li = flow.second.second;
        for (int j = 1; j <= K; j++) {
            for (const auto& node : flowNetwork) {
                const int& u = node.first;
                for (const auto& v : node.second) {
                    MPConstraint *const C31_constraint = solver->MakeRowConstraint(-solver->infinity(), 0.0, "C31_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                    C31_constraint->SetCoefficient(f[{i, j, u, v}], 1);
                    C31_constraint->SetCoefficient(b[{i, j, u, v}], -1 * li);
                }
            }
        }
        ++i;
    }
    
    // C3-2: Avoiding branching flows and ensuring simple paths (for each flow-split, there can be only one outgoing binary flow-split
    // occupancy variable bij(u,v) from any router)
    i = 1;  
    for (const auto& flow : flows) {
        for (int j = 1; j <= K; j++) {
            for (const auto& node : flowNetwork) {
                const int& u = node.first;
                MPConstraint *const C32_constraint = solver->MakeRowConstraint(0.0, 1.0, "C32_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u));
                const auto& vec = node.second;
                for (const auto& v : vec)
                    C32_constraint->SetCoefficient(b[{i, j, u, v}], 1);
            }
        }
        ++i;
    }

    // -------------------------------------------------------------------------------------------------------------------------------

    // ---------------------------------- C4 -----------------------------------------------------------------------------------------
    // ensuring source-to-destination connectivity by avoiding meaningless solutions in which the source and destination
    // have self-loops without a contiguous path between them
    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first, di = flow.second.first;
        for (int j = 1; j <= K; j++) {
            if (reverseFlowNetwork.find(si) != reverseFlowNetwork.end()) {
                for (const auto& u : reverseFlowNetwork.at(si)) {
                    MPConstraint *const C41_constraint = solver->MakeRowConstraint(0.0, 0.0, "C41_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(si));
                    C41_constraint->SetCoefficient(f[{i, j, u, si}], 1);
                }
            }
            
            if (flowNetwork.find(di) != flowNetwork.end()) {
                for (const auto& v : flowNetwork.at(di)) {
                    MPConstraint *const C42_constraint = solver->MakeRowConstraint(0.0, 0.0, "C42_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(di) + "_" + std::to_string(v));
                    C42_constraint->SetCoefficient(f[{i, j, di, v}], 1);
                }
            }
        }
        ++i;
    }

    // -------------------------------------------------------------------------------------------------------------------------------

    // ---------------------------------- Objectives -----------------------------------------------------------------------------------------
    // Possible Objectives / Cost Functions
    // 1] O1 : minimize the maximum channel load (MCL)
    // This can be done regardless of network capacity, and only knowing the relative demands of flows.
    // This objective is originally formulated as : minimize max(h(v), ∀(u, v) ∈ E) . This contains a max operation, which is a non-linear operation.
    // To linearize this objective, assume some auxiliary variable Z = max(h(v), ∀(u, v) ∈ E)
    // The equivalent linear formulation is: minimize Z, subject to Z >= h(v), ∀(u, v) ∈ E --> h(v) - Z <= 0
    // h(v) = summation(i = 1 to m), summation (j = 1 to k), fij(u, v)
    
    // O1
    operations_research::MPVariable* const Z = solver->MakeIntVar(0.0, solver->infinity(), "O1"); // can this be continuous?
    for (const auto& node : flowNetwork) {
        const int& u = node.first;
        for (const auto & v : node.second) {
            std::string constraintName = "O1C_" + std::to_string(u) + "_" + std::to_string(v);
            MPConstraint* const O1C_constraint = solver->MakeRowConstraint(-solver->infinity(), 0.0, constraintName);
            O1C_constraint->SetCoefficient(Z, -1);
            int i = 1;
            for (const auto& flow : flows) {
                for (int j = 1; j <= K; ++j)
                    O1C_constraint->SetCoefficient(f[{i, j, u, v}], 1);
                ++i;
            }    
        }    
    }

    // O2: number of active Channels
    operations_research::MPVariable* const O = solver->MakeIntVar(0.0, solver->infinity(), "O2"); // the number of channel is always discrete
    std::string constraintName = "O2C";
    MPConstraint* const O2C_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);
    O2C_constraint->SetCoefficient(O, -1);
    for (const auto& node : flowNetwork) {
        const int& u = node.first;
        for (const auto& v : node.second) {
            int i = 1;
            for (const auto& flow : flows) {
                for (int j = 1; j <= K; ++j)
                    O2C_constraint->SetCoefficient(b[{i, j, u, v}], 1);
                ++i;
            }
        }
    }

    // combined objective giving higher priority to Z over O
    operations_research::MPObjective* const objective = solver->MutableObjective();
    objective->SetCoefficient(Z, 1e6);   // higher priority objective
    objective->SetCoefficient(O, 1.0);      // lower priority objective
    objective->SetMinimization();
    // -------------------------------------------------------------------------------------------------------------------------------
    
    solver->EnableOutput(); // enable solver's own log output
    // Solve the problem
    const MPSolver::ResultStatus result_status = solver->Solve();

    std::stringstream solverLog;
    milp_solver.getSolutionLog(result_status, objective, solverLog);

    writeLogToFile(solverLog, "app/output/milps/txcomm_milp_output.txt");
    WriteVariablesToFile(*solver, "app/output/milps/txcomm_milp_output.txt", "append");
}

void solve_transcom_fission_fusion_MILP(
    MILPSolver& milp_solver,
    std::unordered_map<int, std::pair<std::unordered_set<int>, double>> &flows, 
    std::unordered_map<int, std::vector<int>> &flowNetwork, 
    std::unordered_map<int, std::vector<int>> &reverseFlowNetwork,
    std::unordered_map<std::pair<int, int>, int, pair_hash> &min_hops
    )
    {

    // Retrieve the solver
    MPSolver* solver = milp_solver.get();

    // Degree of fission is always 4
    int K = 4;

    // Define the Variables
    int M = (int)flows.size();
    std::map<std::tuple<int, int, int, int>, MPVariable*> f; // fi,j(u,v) : flow variable for split j of flow i on edge (u,v)
    std::map<std::tuple<int, int, int, int>, MPVariable*> b; // bi,j(u,v) : boolean variable for split j of flow i on edge (u,v)
    std::map<std::tuple<int, int, int, int>, MPVariable*> p; // pi,j(u,v) : boolean variable for split j of flow i on edge (u,v)
    
    int i = 1;
    for (const auto& flow : flows) {
        const int si = flow.first;
        const auto& ti_set = flow.second.first;  // This is now a set of destinations
        const double li = flow.second.second;

        std::vector<std::pair<int, int>> TraversibleEdges;
        std::unordered_map<int, bool> isVisited;
        std::unordered_map<int, bool> leadsToAnyTi;
        
        // For each destination in ti_set, find paths from si
        for (const int ti : ti_set) {
            std::unordered_map<int, bool> currentVisited;
            std::unordered_map<int, bool> currentLeadsToTi;
            generateTraversibleEdges(ti, si, flowNetwork, currentVisited, currentLeadsToTi, TraversibleEdges);
            
            // Merge the leadsTo information
            for (const auto& [vertex, leads] : currentLeadsToTi) {
                if (leads) leadsToAnyTi[vertex] = true;
            }
        }
        
        // Remove duplicate edges (optional but recommended)
        std::sort(TraversibleEdges.begin(), TraversibleEdges.end());
        TraversibleEdges.erase(std::unique(TraversibleEdges.begin(), TraversibleEdges.end()), TraversibleEdges.end());
        
        // Create variables as before
        for (const auto& edge : TraversibleEdges) {
            const int &u = edge.first, &v = edge.second;
            for (int j = 1; j <= K; j++) {
                f[{i, j, u, v}] = solver->MakeNumVar(0.0, li, "f_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                b[{i, j, u, v}] = solver->MakeBoolVar("b_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                p[{i, j, u, v}] = solver->MakeBoolVar("p_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
            }
        }
        
        ++i;
    }

    i = 1;
    std::map<std::tuple<int, int>, MPVariable*> q; // qi,j : the flow on the j-th split on the i-th multicast tree
    for (const auto& flow : flows) {
        const double li = flow.second.second;
        for (int j = 1; j <= K; j++) {
            std::string flowVarName = "q_" + std::to_string(i) + "_" + std::to_string(j);
            q[{i, j}] = solver->MakeNumVar(0.0, li, flowVarName);
        }
        ++i;
    }

    // Define the constraints
    
    
    // ---------------------------------- C1 -----------------------------------------------------------------------------------------
    // Because multicast data is replicated at such branching routers, flow-conservation does not hold. 
    // Instead, the flow can potentially increase at such branching routers
    // This is expressed in constraint by constraining the outgoing flows at router v to exceed or equal the incoming flows at the same node.

    i = 1;
    for (const auto& flow: flows) {
        const int& si = flow.first;
        const auto& Di = flow.second.first;
        for (int j = 1; j <= K; ++j) {
            for (const auto & node : flowNetwork) {
                const int& v = node.first;
                if (v == si || Di.find(v) != Di.end()) continue;
                std::string constraintName = "C1_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v);
                MPConstraint *const C1_constraint = solver->MakeRowConstraint(0.0, solver->infinity(), constraintName);
                const auto& vec = node.second;
                for (const auto& u : vec) C1_constraint->SetCoefficient(f[{i, j, v, u}], 1); // outgoing flow
                
                if (reverseFlowNetwork.find(v) != reverseFlowNetwork.end()) {
                    const auto& rev_vec = reverseFlowNetwork.at(v);
                    for (const auto& u : rev_vec) C1_constraint->SetCoefficient(f[{i, j, u, v}], -1); // incoming flow
                }
            }
        }
        ++i;
    }
    // ------------------------------------------------------------------------------------------------------------------------------
    
    
    
    // ---------------------------------- C2 -----------------------------------------------------------------------------------------
    // C2
    // For multicast trees: 
    // At each intermediate node (not source or destination),
    // sum of incoming occupancy b(w,u) >= sum of outgoing occupancy b(u,v)

    // b_i_j(u,v)  ≤  Σ_{(w,u)∈E} b_i_j(w,u)
    // f_i_j(u,v)  ≤  Σ_{(w,u)∈E} f_i_j(w,u)

    i = 1;
    for (const auto& flow : flows) {
        const int  si = flow.first;
        const auto& Di = flow.second.first;

        for (int j = 1; j <= K; ++j) {

            for (const auto& node : flowNetwork) {
                const int u = node.first;
                if (u == si || Di.find(u) != Di.end()) continue;   // skip source & dests

                // Pre-compute list of predecessors once per u
                if (reverseFlowNetwork.find(u) == reverseFlowNetwork.end()) continue;
                const auto& preds = reverseFlowNetwork.at(u);

                for (const int v : node.second) {                  // every outgoing edge (u,v)
                    // ----- binary occupancy inequality -----
                    {
                        std::string cname = "C2b_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                        MPConstraint* C2b = solver->MakeRowConstraint(-solver->infinity(), 0.0, cname);
                        C2b->SetCoefficient(b[{i,j,u,v}], 1);      // + b(u,v)
                        for (int w : preds) C2b->SetCoefficient(b[{i,j,w,u}], -1); // – Σ_in b(w,u)
                    }

                    // ----- flow inequality (optional but literal-spec) -----
                    {
                        std::string cname = "C2f_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                        MPConstraint* C2f = solver->MakeRowConstraint( -solver->infinity(), 0.0, cname);
                        C2f->SetCoefficient(f[{i,j,u,v}], 1);      // + f(u,v)
                        for (int w : preds) C2f->SetCoefficient(f[{i,j,w,u}], -1); // – Σ_in f(w,u)
                    }
                }
            }
        }
        ++i;
    }
    // ------------------------------------------------------------------------------------------------------------------------------
    
    
    
    // ---------------------------------- C3 -----------------------------------------------------------------------------------------
    // C3-1: Avoiding negative flow values by ensuring that no flow-split exceeds the unsplit flow
    
    i = 1;
    for (const auto& flow: flows) {
        double li = flow.second.second;
        for (int j = 1; j <= K; j++) {
            for (const auto& node : flowNetwork) {
                const int& u = node.first;
                for (const auto& v : node.second) {
                    std::string constraintName = "C31_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u)
                    + "_" + std::to_string(v);
                    MPConstraint *const C31_constraint = solver->MakeRowConstraint(-solver->infinity(), 0.0, constraintName);
                    C31_constraint->SetCoefficient(f[{i, j, u, v}], 1);
                    C31_constraint->SetCoefficient(b[{i, j, u, v}], -1 * li);
                }
            }
        }
        ++i;
    }
    // ------------------------------------------------------------------------------------------------------------------------------
    
    
    
    // ---------------------------------- C3 -----------------------------------------------------------------------------------------
    // C3-2: Limit incoming occupancy at each node (except source) to at most one
    i = 1;
    for (const auto& flow : flows) {
        const int& si = flow.first;
        for (int j = 1; j <= K; ++j) {
            for (const auto& node : flowNetwork) {
                const int& v = node.first;
                if (v == si) continue; // Don't limit the source

                std::string constraintName = "C32_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v);
                MPConstraint* const C32_constraint = solver->MakeRowConstraint(0.0, 1.0, constraintName);

                if (reverseFlowNetwork.find(v) != reverseFlowNetwork.end()) {
                    for (const auto& u : reverseFlowNetwork.at(v)) {
                        C32_constraint->SetCoefficient(b[{i, j, u, v}], 1);
                    }
                }
            }
        }
        ++i;
    }
    
    
    // ------------------------------------------------------------------------------------------------------------------------------

    
    
    // ---------------------------------- C4 -----------------------------------------------------------------------------------------
    
    // Optimized C4
    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first;
        for (int j = 1; j <= K; j++) {
            std::string constraintName = "C4_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(si);
            MPConstraint* const C4_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);

            if (reverseFlowNetwork.find(si) != reverseFlowNetwork.end()) {
                for (const auto& u : reverseFlowNetwork.at(si)) {
                    C4_constraint->SetCoefficient(f[{i, j, u, si}], 1);
                }
            }
        }
        ++i;
    }
    
    // ---------------------------------- C5 -----------------------------------------------------------------------------------------
    // ensures thas the total outgoing load at the source equals or exceeds the original stream load 
    // (because of branching at the source, the outgoing load may exceed the original load)
    
    // C5-1: ∀i sum_{(si,v)∈E sum_{j=1 to K}}f_i_j(si,v) >= li, where li is the flow demand of flow i
    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first;
        double li = flow.second.second;
        std::string constraintName = "C5_" + std::to_string(i);
        MPConstraint *const C51_constraint = solver->MakeRowConstraint(li, solver->infinity(), constraintName);
        for (int j = 1; j <= K; j++) {
            if (flowNetwork.find(si) == flowNetwork.end()) continue;
            for (const auto& v : flowNetwork.at(si)) {
                C51_constraint->SetCoefficient(f[{i, j, si, v}], 1);
            }
        }
        ++i;
    }

    // C5-2: sum_{j=1 to K} q_i_j = li (sum_{j=1 to K} f_i_j(si, v) = l_i)
    // This ensures that the total flow across all splits j equals the load li
    i = 1;
    for (const auto& flow : flows) {
        double li = flow.second.second;
        std::string constraintName = "C52_" + std::to_string(i);
        MPConstraint* C52 = solver->MakeRowConstraint(li, li, constraintName);
        for (int j = 1; j <= K; ++j) {
            C52->SetCoefficient(q[{i, j}], 1);
        }
        ++i;
    }

    // ------------------------------------------------------------------------------------------------------------------------------
    
    
    
    // ---------------------------------- C6 -----------------------------------------------------------------------------------------

    // C6: ∀i, ∀j, ∀v ∈ Di sum_{(u,v)∈E} f_i_j(u,v) = q_i_j
    // This ensures that the total flow into each destination v ∈ Di equals the flow-split amount q_i_j
    i = 1;
    for (const auto& flow : flows) {
        const auto& Di = flow.second.first; // Destinations for flow i
        for (int j = 1; j <= K; ++j) { // Iterate over all paths j
            for (const auto& v : Di) { // Iterate over all destinations v ∈ Di
                std::string constraintName = "C6_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v);
                MPConstraint* C6 = solver->MakeRowConstraint(0, 0, constraintName);
                if (reverseFlowNetwork.find(v) != reverseFlowNetwork.end()) {
                    for (const auto& u : reverseFlowNetwork.at(v)) { // Sum over all (u,v) ∈ E
                        C6->SetCoefficient(f[{i, j, u, v}], 1); // Sum over f_i_j(u,v)
                    }
                }
                C6->SetCoefficient(q[{i, j}], -1); // Subtract q_i_j
            }
        }
        ++i; // Increment flow index
    }


    // ------------------------------------------------------------------------------------------------------------------------------
    
    
    
    // ---------------------------------- C7 -----------------------------------------------------------------------------------------

    // C7: ∀i, ∀j, ∀(u,v)∈E, u!=si f_i_j(u,v) = q_i_j * b_i_j(u,v)
    // This is a product of variables constraint, which is non-linear. To linearize it, we introduce a new variable z[i,j,u,v] = q_i_j * b[i,j,u,v]

    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first; // Source node for flow i
        double li = flow.second.second; // Flow demand of flow i
        double M = li;  // Define a sufficiently large constant M: the max value a flow variable f[i,j,u,v] can take is the full flow demand li
        for (int j = 1; j <= K; ++j) { // Iterate over all paths j
            for (const auto& node : flowNetwork) { // Iterate over all nodes u in the network
                const int& u = node.first;
                if (u == si) continue; // Skip if u is the source node
                for (const auto& v : node.second) { // Iterate over all edges (u,v) ∈ E
                    // Create new variable z[i,j,u,v] = q_i_j * b[i,j,u,v]
                    MPVariable* z = solver->MakeNumVar(0, M, "z_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));

                    // Constraint: z[i,j,u,v] <= M * b[i,j,u,v]
                    // formulation: -inf < z[i,j,u,v] - M * b[i,j,u,v] <= 0                
                    std::string constraintName1 = "C7_upper1_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C7_upper1 = solver->MakeRowConstraint(-solver->infinity(), 0, constraintName1);
                    C7_upper1->SetCoefficient(z, 1);
                    C7_upper1->SetCoefficient(b[{i, j, u, v}], -M);

                    // Constraint: z[i,j,u,v] <= q_i_j
                    // formulation: -inf <= z[i,j,u,v] - q_i_j <= 0
                    std::string constraintName2 = "C7_upper2_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C7_upper2 = solver->MakeRowConstraint(-solver->infinity(), 0, constraintName2);
                    C7_upper2->SetCoefficient(z, 1);
                    C7_upper2->SetCoefficient(q[{i, j}], -1);

                    // Constraint: z[i,j,u,v] >= q_i_j - M * (1 - b[i,j,u,v])
                    // formulation: -M <= z[i,j,u,v] - q_i_j + M * b[i,j,u,v] < inf
                    std::string constraintName3 = "C7_lower_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C7_lower = solver->MakeRowConstraint(-M, solver->infinity(), constraintName3);
                    C7_lower->SetCoefficient(z, 1);
                    C7_lower->SetCoefficient(q[{i, j}], -1);
                    C7_lower->SetCoefficient(b[{i, j, u, v}], -M);

                    // Constraint: z[i,j,u,v] >= 0
                    // formulation: 0 <= z[i,j,u,v] < inf
                    std::string constraintName4 = "C7_nonneg_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C7_nonneg = solver->MakeRowConstraint(0, solver->infinity(), constraintName4);
                    C7_nonneg->SetCoefficient(z, 1);

                    // Constraint: f[i,j,u,v] = z[i,j,u,v]
                    std::string constraintName5 = "C7_eq_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C7_eq = solver->MakeRowConstraint(0, 0, constraintName5);
                    C7_eq->SetCoefficient(f[{i, j, u, v}], 1);
                    C7_eq->SetCoefficient(z, -1);
                }
            }
        }
        ++i; // Increment flow index
    }
    // ------------------------------------------------------------------------------------------------------------------------------
    
    
    
    // ---------------------------------- C8 -----------------------------------------------------------------------------------------

    // C8-1: ∀i, ∀j, ∀(si,v)∈E p_i_j(si,v) = b_i_j(si,v)

    i = 1;
    for (const auto& flow: flows) {
        int si = flow.first;
        if (flowNetwork.find(si) == flowNetwork.end()) continue;
        for (const auto& v : flowNetwork.at(si)) {
            for (int j = 1; j <= K; ++j) {
                std::string constraintName = "C81_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v);
                MPConstraint *const C81_Constraint = solver->MakeRowConstraint(0, 0, constraintName);
                C81_Constraint->SetCoefficient(p[{i, j, si, v}], 1);
                C81_Constraint->SetCoefficient(b[{i, j, si, v}], -1);
            }
        }
        ++i;
    }

    
    // C8-2: ∀i, ∀j, ∀(u,v)∈E p_i_j(u,v) = {[summation over (w,u)∈E p_i_j(w,u) + 1 if b_i_j(u,v)] = 1, or [0 if b_i_j(u,v) = 0]}

    i = 1;
    for (const auto& flow : flows) {
        // Define a sufficiently large constant M: the maximum possible value of sum_{(w,u)∈E}p[i,j,u,v]+1
        double M = 1e4; // Adjust this value based on your problem: M = K * (max_possible_path_length + 1)
        for (int j = 1; j <= K; ++j) { // Iterate over all paths j
            for (const auto& node : flowNetwork) { // Iterate over all nodes u in the network
                const int& u = node.first;
                for (const auto& v : node.second) { // Iterate over all edges (u,v) ∈ E
                    // Create constraint: p[i,j,u,v] - sum_{(w,u)∈E} p[i,j,w,u] - M * (1 - b[i,j,u,v]) <= 1
                    // formulation: -inf < p[i,j,u,v] - sum_{(w,u)∈E} p[i,j,w,u] - M * b[i,j,u,v] <= 1 + M
                    std::string constraintName1 = "C82_upper_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C82_upper = solver->MakeRowConstraint(-solver->infinity(), 1 + M, constraintName1);
                    C82_upper->SetCoefficient(p[{i, j, u, v}], 1);
                    if (reverseFlowNetwork.find(u) != reverseFlowNetwork.end()) {
                        for (const auto& w : reverseFlowNetwork.at(u)) { // Sum over all (w,u) ∈ E
                            C82_upper->SetCoefficient(p[{i, j, w, u}], -1);
                        }
                    }
                    C82_upper->SetCoefficient(b[{i, j, u, v}], M);

                    // Create constraint: p[i,j,u,v] - sum_{(w,u)∈E} p[i,j,w,u] + M * (1 - b[i,j,u,v]) >= 1
                    // formulation: 1 - M <= p[i,j,u,v] - sum_{(w,u)∈E} p[i,j,w,u] - M * b[i,j,u,v] <= inf
                    std::string constraintName2 = "C82_lower_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C82_lower = solver->MakeRowConstraint(1 - M, solver->infinity(), constraintName2);
                    C82_lower->SetCoefficient(p[{i, j, u, v}], 1);
                    if (reverseFlowNetwork.find(u) != reverseFlowNetwork.end()) {
                        for (const auto& w : reverseFlowNetwork.at(u)) { // Sum over all (w,u) ∈ E
                            C82_lower->SetCoefficient(p[{i, j, w, u}], -1);
                        }   
                    }
                    C82_lower->SetCoefficient(b[{i, j, u, v}], -M);

                    // Create constraint: p[i,j,u,v] <= M * b[i,j,u,v]
                    // formulation: -inf <= p[i,j,u,v] - M * b[i,j,u,v] <= 0
                    std::string constraintName3 = "C82_zero_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C82_zero = solver->MakeRowConstraint(-solver->infinity(), 0, constraintName3);
                    C82_zero->SetCoefficient(p[{i, j, u, v}], 1);
                    C82_zero->SetCoefficient(b[{i, j, u, v}], -M);
                }
            }
        }
        ++i; // Increment flow index
    }

    // ------------------------------------------------------------------------------------------------------------------------------


    // ---------------------------------- C9 -----------------------------------------------------------------------------------------
    // ∀i, ∀j, ∀t, summation over (u, D_i_t)∈E p_i_j(u,D_i_t) >= Min_HOPS(s_i, D_i_t)

    i = 1;
    for (const auto& flow: flows) {
        int si = flow.first;
        const auto& Di = flow.second.first;
        for (int j = 1; j <= K; j++) {
            for (const auto& t: Di) {
                MPConstraint *const C9_Constraint = solver->MakeRowConstraint(min_hops[{si, t}], solver->infinity(), "C9_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(si) + "_" + std::to_string(t));
                if (reverseFlowNetwork.find(t) == reverseFlowNetwork.end()) continue;
                for (const auto& u : reverseFlowNetwork.at(t))
                    C9_Constraint->SetCoefficient(p[{i, j, u, t}], 1);
            }
        }
        ++i;
    }
    // ------------------------------------------------------------------------------------------------------------------------------

    // Possible Objectives / Cost Functions
    // 1] O1 : minimize the maximum channel load (MCL)
    // This can be done regardless of network capacity, and only knowing the relative demands of flows.
    // This objective is originally formulated as : minimize max(h(v), ∀(u, v) ∈ E) . This contains a max operation, which is a non-linear operation.
    // To linearize this objective, assume some auxiliary variable Z = max(h(v), ∀(u, v) ∈ E)
    // The equivalent linear formulation is: minimize Z, subject to Z >= h(v), ∀(u, v) ∈ E --> h(v) - Z <= 0
    // h(v) = summation(i = 1 to m), summation (j = 1 to k), fij(u, v)
    
    // O1
    operations_research::MPVariable* const Z = solver->MakeNumVar(0.0, solver->infinity(), "O1");
    for (const auto& node : flowNetwork) {
        const int& u = node.first;
        for (const auto & v : node.second) {
            std::string constraintName = "O1C_" + std::to_string(u) + "_" + std::to_string(v);
            MPConstraint* const O1C_constraint = solver->MakeRowConstraint(-solver->infinity(), 0.0, constraintName);
            O1C_constraint->SetCoefficient(Z, -1);
            int i = 1;
            for (const auto& flow : flows) {
                for (int j = 1; j <= K; ++j)
                    O1C_constraint->SetCoefficient(f[{i, j, u, v}], 1);
                ++i;
            }    
        }    
    }

    // O2
    operations_research::MPVariable* const O = solver->MakeNumVar(0.0, solver->infinity(), "O2");
    std::string constraintName = "O2C";
    MPConstraint* const O2C_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);
    O2C_constraint->SetCoefficient(O, -1);
    for (const auto& node : flowNetwork) {
        const int& u = node.first;
        for (const auto& v : node.second) {
            int i = 1;
            for (const auto& flow : flows) {
                for (int j = 1; j <= K; ++j)
                    O2C_constraint->SetCoefficient(b[{i, j, u, v}], 1);
                ++i;
            }
        }
    }
    // Note: without O2, we get solutions including, for example:
    // b_1_1_1_3 = 1, f_1_1_1_3 = 0   ---  b_1_2_1_3 = 1, f_1_2_1_3 = 0   ---  b_1_3_1_3 = 1, f_3_2_1_3 = 0   ---  b_1_4_1_3 = 1, f_1_4_1_3 = 25
    // Here, it was the right solution that one split (split 4 in this case) takes the full flow load (l1=25), but why are other 
    // b_i_j_1_3 (except j=4) taking values of 1 when there corresponding f_i_j_1_3 take 0 values?
    // This objective eliminates this case, but shouldn't it be eliminated even without it?

    // combined objective giving higher priority to Z over O
    operations_research::MPObjective* const objective = solver->MutableObjective();
    objective->SetCoefficient(Z, 1e6);   // higher priority objective
    objective->SetCoefficient(O, 1.0);      // lower priority objective
    objective->SetMinimization();
    // -------------------------------------------------------------------------------------------------------------------------------

    solver->EnableOutput(); // enable solver's own log output
    // Solve the problem
    const MPSolver::ResultStatus result_status = solver->Solve();

    std::stringstream solverLog;
    milp_solver.getSolutionLog(result_status, objective, solverLog);

    writeLogToFile(solverLog, "app/output/milps/txcomm_fusion_milp_output.txt");
    WriteVariablesToFile(*solver, "app/output/milps/txcomm_fusion_milp_output.txt", "append");


    // writeLogToFile(solverLog, "app/output/milps/transcom_fissionFusion_log.txt");
    // writeConstraintsToFile(*solver, "app/output/milps/transcom_fissionFusion_constraints.txt");
    // WriteVariablesToFile(*solver, "app/output/milps/transcom_fissionFusion_variables.txt");
}
}  // namespace operations_research

