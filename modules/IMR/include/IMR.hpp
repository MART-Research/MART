#include "../utils/milps/orUtils.hpp"
#include "../utils/milps/MARTUtility.hpp"
#include "../utils/milps/MILPSolver.hpp"

// Forward declaration of the MILP solver function
namespace operations_research {
    void solve_imr_fission_only_MILP(
        MILPSolver& milp_solver,
        std::multimap<int, std::pair<int, double>> &flows, 
        std::unordered_map<int, std::vector<int>> &flowNetwork, 
        std::unordered_map<int, std::vector<int>> &reverseFlowNetwork,
        unsigned int concentrationFactor,
        unsigned int numNodes
    );

    void solve_imr_fission_fusion_MILP(
        MILPSolver& milp_solver,
        std::unordered_map<int, std::pair<std::unordered_set<int>, double>> &flows, // {src, {Di, demand}}
        std::unordered_map<int, std::vector<int>> &flowNetwork, 
        std::unordered_map<int, std::vector<int>> &reverseFlowNetwork,
        unsigned int concentrationFactor,
        unsigned int numNodes
    );
}

#ifndef IMR_H
#define IMR_H

class IMR {
public:
    IMR // Constructor for fission only version
    (
        MILPSolver& solver,
        std::unordered_map<int, std::vector<int>> &flowNetwork,
        std::multimap<int, std::pair<int, double>> &communicationGraph,
        int num_nodes,
        int concentration_factor = 1
    ) : 
    milpSolver(solver),
    flowNetwork(flowNetwork),
    commGraph(communicationGraph),
    numNodes(num_nodes),
    concentrationFactor(concentration_factor)
    {}

    IMR // Constructor for fission + fusion version
    (
        MILPSolver& solver,
        std::unordered_map<int, std::vector<int>> &flowNetwork,
        std::unordered_map<int, std::pair<std::unordered_set<int>, double>> communicationGraph, 
        int num_nodes,
        int concentration_factor = 1
    ) :
    milpSolver(solver),
    numNodes(num_nodes),
    concentrationFactor(concentration_factor),
    flowNetwork(flowNetwork),
    commGraphFusion(communicationGraph)
    {}

    void run_imr_fission_only() {
        // Create the reverse flow network
        for (const auto& vertex : flowNetwork) {
            int src = vertex.first;
            const auto& vec = vertex.second;
            for (const auto& dst : vec) {
                reverseFlowNetwork[dst].emplace_back(src);
            }
        }

        operations_research::solve_imr_fission_only_MILP(milpSolver, commGraph, flowNetwork, reverseFlowNetwork, concentrationFactor, numNodes); // produces flows and saves them to a file
    }

    void run_imr_fission_fusion() {

        // Create the reverse flow network
        for (const auto& vertex : flowNetwork) {
            int src = vertex.first;
            const auto& vec = vertex.second;
            for (const auto& dst : vec) {
                reverseFlowNetwork[dst].emplace_back(src);
            }
        }

        operations_research::solve_imr_fission_fusion_MILP(milpSolver, commGraphFusion, flowNetwork, reverseFlowNetwork, concentrationFactor, numNodes); // produces flows and saves them to a file
    }
    
private:
    MILPSolver& milpSolver;
    int numNodes;
    int concentrationFactor;
    std::unordered_map<int, std::vector<int>> flowNetwork;
    std::multimap<int, std::pair<int, double>> commGraph;  // The flows, each flow is represented as {src, {dest, demand}}
    std::unordered_map<int, std::pair<std::unordered_set<int>, double>> commGraphFusion;  // The flows, each flow is represented as {src, {Di, demand}}, where Di = set of destinations
    std::unordered_map<int, std::vector<int>> reverseFlowNetwork;
};
#endif
namespace operations_research {
void solve_imr_fission_only_MILP(
    MILPSolver& milp_solver,
    std::multimap<int, std::pair<int, double>> &flows,  
    std::unordered_map<int, std::vector<int>> &flowNetwork, 
    std::unordered_map<int, std::vector<int>> &reverseFlowNetwork,
    unsigned int concentrationFactor,
    unsigned int numNodes
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
    std::map<std::tuple<int, int, int>, MPVariable*> h_s; // hi,j,si,v : slack variable for source
    std::map<std::tuple<int, int, int>, MPVariable*> h_d; // hi,j,di,v : slack variable for destination

    std::vector<std::pair<int, int>> TraversibleEdges = getAllEdges(flowNetwork);
    int i = 1;
    for (const auto& flow : flows) {
        const int si = flow.first;
        const int ti = flow.second.first;
        const double li = flow.second.second;

        for (const auto& [u, v] : TraversibleEdges) {
            if (u == ti || v == si) continue; // for flow i, we don't need any flow from the destination ti or into the source si
            for (int j = 1; j <= K; j++) {
                f[{i, j, u, v}] = solver->MakeNumVar(0.0, li, "f_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                b[{i, j, u, v}] = solver->MakeBoolVar("b_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
            }
        }
        
        // Create needed slack variables        
        std::vector<int> allRouters; allRouters.reserve(numNodes); for (int v = 0; v < numNodes; ++v) allRouters.push_back(v);
        for (const auto& v: allRouters) {
            for (int j = 1; j <= K; j++) {
                h_s[{i, j, v}] = solver->MakeNumVar(0.0, li, "h_s_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v));
                h_d[{i, j, v}] = solver->MakeNumVar(0.0, li, "h_d_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v));
            }
        }
        ++i;
    }

    // Thread mapping variable: g_a_v: binary variable indicating whether thread a is mapped to vertex v
    std::map<std::pair<int, int>, MPVariable*> g;
    std::set<int> threads;
    // Initialize threads with source and destination threads
    for (const auto& flow : flows) {
        std::cout << flow.first << " -> " << flow.second.first << " : " << flow.second.second << std::endl;
        threads.insert(flow.first);          // Source
        threads.insert(flow.second.first);   // Destination
    }
    for (int a : threads) {
        for (int v = 0; v < numNodes; ++v){
            g[{a, v}] = solver->MakeBoolVar("g_" + std::to_string(a) + "_" + std::to_string(v));
        }
    }

    // Define the constraints
    // ----- C1 starts here -----
    // C1: Each thread must be mapped to exactly one vertex
    for (int a : threads) {
        std::string constraintName = "C11_" + std::to_string(a);
        MPConstraint* c1 = solver->MakeRowConstraint(1, 1, constraintName);
        for (int v = 0; v < numNodes; ++v){
            c1->SetCoefficient(g[{a, v}], 1);
        }
    }
    
    // C1: Each vertex can host at most concentrationFactor threads
    for (int v = 0; v < numNodes; ++v) {
        std::string constraintName = "C12_" + std::to_string(v);
        MPConstraint* c1_v = solver->MakeRowConstraint(0, concentrationFactor, constraintName);
        for (int a : threads) {
            c1_v->SetCoefficient(g[{a, v}], 1);
        }
    }
    // ----- C1 ends here -----

    // ----- C2 starts here -----
    // C2: Flow conservation constraints with slack variables
    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first, di = flow.second.first;
        double li = flow.second.second;
        for (int j = 1; j <= K; ++j) {
            for (const auto& node : flowNetwork) {
                int v = node.first;
                MPConstraint* c2 = solver->MakeRowConstraint(0, 0, "C2_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v));
                // Slack for source (h_{i,j,si,v}) if v hosts si
                c2->SetCoefficient(h_s.at({i, j, v}), 1);
                // Incoming edges (sum_{(u,v)} f_{i,j}(u,v))
                for (const auto& u : reverseFlowNetwork.at(v)) { c2->SetCoefficient(f[{i, j, u, v}], 1); }
                // Slack for destination (h_{i,j,di,v}) if v hosts di:
                c2->SetCoefficient(h_d.at({i, j, v}), -1);
                // Outgoing edges (sum_{(v,w)} f_{i,j}(v,w))
                for (const auto& w : flowNetwork.at(v)) { c2->SetCoefficient(f[{i, j, v, w}], -1); }
            }
        }
        ++i;
    }
    // ----- C2 ends here -----

    // ----- C3 starts here -----
    // C3: Source/Destination Flow Constraints
    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first;            // Source thread for flow i
        int di = flow.second.first;     // Destination thread for flow i
        double li = flow.second.second; // Flow amount

        for (const auto& node : flowNetwork) {
            int v = node.first;

            // C3a: Source constraint (sum_j h_{i,j,si,v} = li * g_{si,v})
            MPConstraint* c3_s = solver->MakeRowConstraint(0, 0, "C3_s_" + std::to_string(i) + "_" + std::to_string(v));
            for (int j = 1; j <= K; ++j) {
                c3_s->SetCoefficient(h_s.at({i, j, v}), 1);  // Sum slack for source splits
            }
            c3_s->SetCoefficient(g[{si, v}], -li);      // Tie to mapping of si to v

            // C3b: Destination constraint (sum_j h_{i,j,di,v} = li * g_{di,v})
            MPConstraint* c3_d = solver->MakeRowConstraint(0, 0, "C3_d_" + std::to_string(i) + "_" + std::to_string(v));
            for (int j = 1; j <= K; ++j) {
                c3_d->SetCoefficient(h_d.at({i, j, v}), 1);   // Sum slack for destination splits
            }
            c3_d->SetCoefficient(g[{di, v}], -li);      // Tie to mapping of di to v
        }
        ++i;
    }
    
    // ----- C3 ends here -----

    // ----- C4 starts here -----
    // C4: Flow capacity and split constraints
    i = 1;
    for (const auto& flow : flows) {
        double li = flow.second.second; // Flow amount

        for (int j = 1; j <= K; ++j) {
            // C4a: f_{i,j}(u,v) <= li * b_{i,j}(u,v) for all edges (u,v)
            for (const auto& node : flowNetwork) { // Iterate over all nodes u in the network
                const int& u = node.first;
                for (const auto& v : node.second) { // Iterate over all edges (u,v) ∈ E
                    std::string constraintName = "C4a_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* c4a = solver->MakeRowConstraint(-solver->infinity(), 0.0, constraintName);
                    c4a->SetCoefficient(f[{i, j, u, v}], 1);
                    c4a->SetCoefficient(b[{i, j, u, v}], -li);
                }
            }

            // C4b: sum_{(v,w)} b_{i,j}(v,w) <= 1 for all routers v
            for (const auto& node : flowNetwork) {
                int v = node.first;
                std::string constraintName = "C4b_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v);
                MPConstraint* c4b = solver->MakeRowConstraint(0.0, 1.0, constraintName);
                for (const auto& w : flowNetwork.at(v)) {
                    c4b->SetCoefficient(b[{i, j, v, w}], 1);
                }
            }
        }
        ++i;
    }

    // ----- C4 ends here -----

    // Define the objectives
    // O1: Minimize the maximum flow on any edge (Z = max_{(u,v)∈E} sum_{i,j} f_{i,j}(u,v))
    // Linearize: Z >= sum_{i,j} f_{i,j}(u,v) for all (u,v)∈E
    // Z - sum_{i,j} f_{i,j}(u,v) >= 0, for all (u,v)∈E
    MPVariable* Z = solver->MakeNumVar(0.0, solver->infinity(), "O1");
    for (const auto& node : flowNetwork) { // Iterate over all nodes u in the network
        const int& u = node.first;
        for (const auto& v : node.second) { // Iterate over all edges (u,v) ∈ E
            MPConstraint* o1_edge = solver->MakeRowConstraint(0.0, solver->infinity(), "O1_constraint_" + std::to_string(u) + "_" + std::to_string(v));
            o1_edge->SetCoefficient(Z, 1);
            for (int i = 1; i <= M; ++i) {
                for (int j = 1; j <= K; ++j) {
                    o1_edge->SetCoefficient(f[{i, j, u, v}], -1);           
                }
            }
        }
    }

    // O2: Minimize the total number of flit hops (O = sum_{(u,v)} sum_{i,j} b_{i,j}(u,v))
    MPVariable* O = solver->MakeNumVar(0.0, solver->infinity(), "O2");
    MPConstraint* o2 = solver->MakeRowConstraint(0.0, 0.0, "O2_constraint");
    o2->SetCoefficient(O, 1);
    for (const auto& node : flowNetwork) { // Iterate over all nodes u in the network
        const int& u = node.first;
        for (const auto& v : node.second) { // Iterate over all edges (u,v) ∈ E
            for (int i = 1; i <= M; ++i) {
                for (int j = 1; j <= K; ++j) {
                    o2->SetCoefficient(b[{i, j, u, v}], -1);  // O = sum_{i,j,u,v} b_{i,j}(u,v)
                }
            }
        }   
    }

    // -- Selecting Proper Coefficients for the objective functions
    /*
    The key idea behind selecting robust coefficients for the weighted sum objective is to ensure strict lexicographic 
    prioritization of the primary objective (minimizing MCL, Z) over the secondary objective (minimizing active channels, O), 
    while avoiding numerical instability. 
    By setting Z's coefficient to 1 and O's coefficient to 1/(Lmax+1), where Lmax is the worst-case total flow, we guarantee that any 
    improvement in Z dominates the objective value, as even the smallest reduction in Z  outweighs the largest possible change in O. 
    This ensures the solver first optimizes Z to its minimal possible value (Z∗) and only then minimizes O within the space of solutions where Z=Z∗.
    The secondary coefficient is scaled to be small enough to preserve this hierarchy but large enough to avoid floating-point precision issues, 
    effectively mimicking exact lexicographic optimization while using a standard weighted-sum solver.

    Another argument is: since O2 is only needed as tie-breaker (See paper for more), we might need to set O_Coff to just a small value (Careful not to cause numerical instability)
    like O_coff = 1e-6, maybe ask the Proferssor about it.
    */
    double L_max = 0.0;
    for (const auto& flow : flows) L_max += flow.second.second;
    double O_coeff = 1.0;
    // -- 

    // Set the objective function
    MPObjective* objective = solver->MutableObjective();
    objective->SetCoefficient(Z, (L_max + 1.0)); // Primary objective: Minimize MCL
    objective->SetCoefficient(O, O_coeff); // Secondary objective: Minimize flit-hops
    objective->SetMinimization();

    solver->EnableOutput(); // enable solver's own log output
    // Solve the problem
    const MPSolver::ResultStatus result_status = solver->Solve();

    std::stringstream solverLog;
    milp_solver.getSolutionLog(result_status, objective, solverLog);
    solverLog << "numThreads = " << threads.size() << std::endl;

    writeLogToFile(solverLog, "app/output/milps/imr_milp_output.txt");
    WriteVariablesToFile(*solver, "app/output/milps/imr_milp_output.txt", "append");

    LOG(INFO) << "DONE!" << std::endl;
}


void solve_imr_fission_fusion_MILP(
    MILPSolver& milp_solver,
    std::unordered_map<int, std::pair<std::unordered_set<int>, double>> &flows, 
    std::unordered_map<int, std::vector<int>> &flowNetwork, 
    std::unordered_map<int, std::vector<int>> &reverseFlowNetwork,
    unsigned int concentrationFactor,
    unsigned int numNodes
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
    std::map<std::pair<int, int>, MPVariable*> q;       // q_{i,j}: flow-split amount
    std::map<std::tuple<int, int, int, int>, MPVariable*> p;  // p_{i,j}(u,v): path counter (for cycle prevention)

    std::vector<std::pair<int, int>> TraversibleEdges = getAllEdges(flowNetwork);
    int i = 1;
    for (const auto& flow : flows) {
        const int si = flow.first;
        const auto& ti_set = flow.second.first;
        const double li = flow.second.second;
        
        // Create variables as before
        for (const auto& [u, v] : TraversibleEdges) {
            for (int j = 1; j <= K; j++) {
                f[{i, j, u, v}] = solver->MakeNumVar(0.0, li, "f_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                b[{i, j, u, v}] = solver->MakeBoolVar("b_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                p[{i, j, u, v}] = solver->MakeBoolVar("p_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
            }
        }

        ++i;
    }

    // Create q variables for the flow-split amount q_{i,j}
    i = 1;
    for (const auto& flow : flows) {
        const double li = flow.second.second;
        for (int j = 1; j <= K; j++) {
            std::string flowVarName = "q_" + std::to_string(i) + "_" + std::to_string(j);
            q[{i, j}] = solver->MakeNumVar(0.0, li, flowVarName);
        }
        ++i;
    }

    // Thread mapping variable: g_a_v: binary variable indicating whether thread a is mapped to vertex v
    std::map<std::pair<int, int>, MPVariable*> g;
    std::set<int> threads;
    // Initialize threads with source and destination threads
    for (const auto& flow : flows) {
        threads.insert(flow.first);          // Source
        for (const auto& dest : flow.second.first) {
            threads.insert(dest); // Add all destinations to the thread set
        }
    }
    for (int a : threads) {
        for (int v = 0; v < numNodes; ++v){
            g[{a, v}] = solver->MakeBoolVar("g_" + std::to_string(a) + "_" + std::to_string(v));
        }
    }

    // Define the constraints
    // ----- C1 starts here -----
    // C1: Each thread must be mapped to exactly one vertex
    for (int a : threads) {
        std::string constraintName = "C11_" + std::to_string(a);
        MPConstraint* c1 = solver->MakeRowConstraint(1, 1, constraintName);
        for (int v = 0; v < numNodes; ++v){
            c1->SetCoefficient(g[{a, v}], 1);
        }
    }
    
    // C1: Each vertex can host at most concentrationFactor threads
    for (int v = 0; v < numNodes; ++v) {
        std::string constraintName = "C12_" + std::to_string(v);
        MPConstraint* c1_v = solver->MakeRowConstraint(0, concentrationFactor, constraintName);
        for (int a : threads) {
            c1_v->SetCoefficient(g[{a, v}], 1);
        }
    }
    // ----- C1 ends here -----

    // ----- C2 starts here -----
    // C2: Multicast Flow Transmission (Branching Allowed)
    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first; // Source thread for flow i
        auto Di = flow.second.first; // Set of destination threads for flow i
        double li = flow.second.second; // Flow amount for flow i
        double big_M = li;
        for (int j = 1; j <= K; ++j) {
            // Source outgoing flow
            for (const auto& node : flowNetwork) {
                int v = node.first;
                // Sum of outgoing flows
                // sum_{(v,w)} f_{i,j}(v,w) >= {q_{i,j} if g_{si,v} = 1, 0 if g_{si,v} = 0}
                // sum_{(v,w)} f_{i,j}(v,w) >= q_{i,j} - BIG_M*(1 - g_{s_i,v})
                // sum_{(v,w)} f_{i,j}(v,w) >= q_{i,j} - BIG_M + BIG_M*g_{s_i,v}
                // sum_{(v,w)} f_{i,j}(v,w) - q_{i,j} + BIG_M - BIG_M*g_{s_i,v} >= 0
                // sum_{(v,w)} f_{i,j}(v,w) - q_{i,j} + -BIG_M*g_{s_i,v} >= -BIG_M
                MPConstraint* c2_src = solver->MakeRowConstraint(-big_M, solver->infinity(), "C2_src_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v));
                for (const auto& w : flowNetwork.at(v)) { c2_src->SetCoefficient(f[{i, j, v, w}], 1); }
                c2_src->SetCoefficient(q[{i, j}], -1);
                c2_src->SetCoefficient(g[{si, v}], -big_M);
            }

            // Destination incoming flow
            for (const auto& node : flowNetwork) {
                int v = node.first;
                // Sum of incoming flows
                // sum_{(u,v)} f_{i,j}(u,v) >= {q_{i,j} if sum_{d∈D_i} g_{d,v} = 1, 0 if sum_{d∈D_i} g_{d,v} = 0}
                // sum_{(u,v)} f_{i,j}(u,v) >= q_{i,j} - BIG_M*(1 - sum_{d∈D_i} g_{d,v})
                // sum_{(u,v)} f_{i,j}(u,v) >= q_{i,j} - BIG_M + BIG_M*sum_{d∈D_i} g_{d,v}
                // sum_{(u,v)} f_{i,j}(u,v) - q_{i,j} + BIG_M - BIG_M*sum_{d∈D_i} g_{d,v} >= 0
                // sum_{(u,v)} f_{i,j}(u,v) - q_{i,j} - BIG_M*sum_{d∈D_i} g_{d,v} >= -BIG_M
                MPConstraint* c2_dst = solver->MakeRowConstraint(-big_M, solver->infinity(), "C2_dst_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v));
                for (const auto& u : reverseFlowNetwork.at(v)) { c2_dst->SetCoefficient(f[{i, j, u, v}], 1); }
                c2_dst->SetCoefficient(q[{i, j}], -1);
                for (auto d : Di) { c2_dst->SetCoefficient(g[{d, v}], -big_M); }
            }
        }
        ++i;
    }

    // ----- C2 ends here -----
    // ----- C3 starts here -----
    // C3: Flow-Split Conservation
    i = 1;
    for (const auto& flow : flows) {
        int li = flow.second.second; // Flow amount for flow i
        MPConstraint* c3 = solver->MakeRowConstraint(li, li, "C3_" + std::to_string(i));
        for (int j = 1; j <= K; ++j) {
            c3->SetCoefficient(q[{i, j}], 1);
        }
        ++i;
    }
    // ----- C3 ends here -----

    // ----- C4 starts here -----
    // C4: ∀i, ∀j, ∀(u,v)∈E f_i_j(u,v) = q_i_j * b_i_j(u,v)
    // This is a product of variables constraint, which is non-linear. To linearize it, we introduce a new variable z[i,j,u,v] = q_i_j * b[i,j,u,v]
    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first; // Source node for flow i
        double li = flow.second.second; // Flow demand of flow i
        double big_M = li;  // Define a sufficiently large constant big_M: the max value a flow variable f[i,j,u,v] can take is the full flow demand li
        for (int j = 1; j <= K; ++j) { // Iterate over all paths j
            for (const auto& node : flowNetwork) { // Iterate over all nodes u in the network
                const int& u = node.first;
                for (const auto& v : node.second) { // Iterate over all edges (u,v) ∈ E
                    // Create new variable z[i,j,u,v] = q_i_j * b[i,j,u,v]
                    MPVariable* z = solver->MakeNumVar(0, big_M, "z_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));

                    // Constraint: z[i,j,u,v] <= big_M * b[i,j,u,v]
                    // formulation: -inf < z[i,j,u,v] - big_M * b[i,j,u,v] <= 0                
                    std::string constraintName1 = "C4_upper1_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C4_upper1 = solver->MakeRowConstraint(-solver->infinity(), 0, constraintName1);
                    C4_upper1->SetCoefficient(z, 1);
                    C4_upper1->SetCoefficient(b[{i, j, u, v}], -big_M);

                    // Constraint: z[i,j,u,v] <= q_i_j
                    // formulation: -inf <= z[i,j,u,v] - q_i_j <= 0
                    std::string constraintName2 = "C4_upper2_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C4_upper2 = solver->MakeRowConstraint(-solver->infinity(), 0, constraintName2);
                    C4_upper2->SetCoefficient(z, 1);
                    C4_upper2->SetCoefficient(q[{i, j}], -1);

                    // Constraint: z[i,j,u,v] >= q_i_j - big_M * (1 - b[i,j,u,v])
                    // formulation: -big_M <= z[i,j,u,v] - q_i_j + big_M * b[i,j,u,v] < inf
                    std::string constraintName3 = "C4_lower_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C4_lower = solver->MakeRowConstraint(-big_M, solver->infinity(), constraintName3);
                    C4_lower->SetCoefficient(z, 1);
                    C4_lower->SetCoefficient(q[{i, j}], -1);
                    C4_lower->SetCoefficient(b[{i, j, u, v}], big_M);

                    // Constraint: z[i,j,u,v] >= 0
                    // formulation: 0 <= z[i,j,u,v] < inf
                    std::string constraintName4 = "C4_nonneg_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C4_nonneg = solver->MakeRowConstraint(0, solver->infinity(), constraintName4);
                    C4_nonneg->SetCoefficient(z, 1);

                    // Constraint: f[i,j,u,v] = z[i,j,u,v]
                    std::string constraintName5 = "C4_eq_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v);
                    MPConstraint* C4_eq = solver->MakeRowConstraint(0, 0, constraintName5);
                    C4_eq->SetCoefficient(f[{i, j, u, v}], 1);
                    C4_eq->SetCoefficient(z, -1);
                }
            }
        }
        ++i; // Increment flow index
    }
    // ----- C4 ends here -----

    // ----- C5 starts here -----
    // C5: No Phantom Sources
    i = 1;
    for (const auto& flow : flows) {
        int si = flow.first; // Source thread for flow i
        for (int j = 1; j <= K; ++j) {
            for (const auto& node : flowNetwork) {
                int v = node.first;
                for (const auto& w : flowNetwork.at(v)) {
                    std::string constraintName = "C5_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v) + "_" + std::to_string(w);
                    MPConstraint* c5 = solver->MakeRowConstraint(-solver->infinity(), 0, constraintName);
                    c5->SetCoefficient(b[{i, j, v, w}], 1);
                    for (const auto& u : reverseFlowNetwork.at(v)) {
                        c5->SetCoefficient(b[{i, j, u, v}], -1);
                    }
                    c5->SetCoefficient(g[{si, v}], -1);
                }
            }
        }
        ++i;
    }
    // ----- C5 ends here -----
    
    // ----- C6 starts here -----
    i = 1;  
    for (const auto& flow : flows) {
        for (int j = 1; j <= K; j++) {
            for (const auto& node : flowNetwork) {
                const int& u = node.first;
                std::string constraintName = "C32_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u);
                MPConstraint *const C32_constraint = solver->MakeRowConstraint(0.0, 1.0, constraintName);
                const auto& vec = node.second;
                for (const auto& v : vec) {
                    C32_constraint->SetCoefficient(b[{i, j, u, v}], 1);
                }
            }
        }
        ++i;
    }
    // ----- C6 ends here -----

    // ----- C7 starts here -----
    // C7: ∀i, ∀j, ∀(u,v)∈E p_i_j(u,v) = {max_{(w,u)∈E} p_i_j(w,u) + 1 if b_i_j(u,v) = 1, or 0 if b_i_j(u,v) = 0}

    // Big-M value (greater than maximum possible path length)
    // const double big_M7 = flowNetwork.size() + 2; // |V| + 2 suffices

    i = 1;
    for (const auto& flow : flows) {
        for (int j = 1; j <= K; ++j) {
            for (const auto& node : flowNetwork) {
                const int& u = node.first;
                for (const auto& v : node.second) {
                    double big_M = 1e5; // Adjust this value later
                    // Handling the MAX operation in the constraint
                    // let Z = max_{(w,u)∈E} p_i_j(w,u) + 1
                    // This is linearized through 3 constraints
                    // 1) Lower Bound Constraint: Z >= p_i_j(w,u) + 1, {w,u)∈E}
                    // 2) Upper Bound Constraint: Z <= p_i_j(w,u) + 1 + big_M*(1 - y(i,j,v,w,u))
                    // 3) Uniqueness Constraint:  Sum_{(w,u)∈E} y(i,j,v,w,u) = 1, {w,u)∈E}
                    
                    MPVariable* const Z = solver->MakeNumVar(0.0, solver->infinity(), "Z_c7_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                    MPConstraint* max_constraint_unique = solver->MakeRowConstraint(1, 1, "c7_max_constraint_uniqueness_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                    for (const auto& w : reverseFlowNetwork.at(u)) {
                        std::map<std::tuple<int, int, int, int, int>, MPVariable*> y;
                        y[{i, j, v, w, u}] = solver->MakeBoolVar("y_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(v) + "_" + std::to_string(w) + "_" + std::to_string(u));
                        // Lower bound constraint
                        // Z >= p_i_j(w,u) + 1 --> Z - p_i_j(w,u) >= 1
                        MPConstraint* max_constraint_lb = solver->MakeRowConstraint(1, solver->infinity(), "c7_max_constraint_lb_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(w) + "_" + std::to_string(u) + "_" + std::to_string(v));
                        max_constraint_lb->SetCoefficient(Z, 1);
                        max_constraint_lb->SetCoefficient(p[{i, j, w, u}], -1);
                        
                        // Upper bound constraint
                        // Z <= p_i_j(w,u) + 1 + big_M*(1 - y(i,j,v,w,u)) --> Z - p_i_j(w,u) + big_M*y(i,j,v,w,u) <= 1 + big_M
                        MPConstraint* max_constraint_ub = solver->MakeRowConstraint(-solver->infinity(), 1 + big_M, "c7_max_constraint_ub_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(w) + "_" + std::to_string(u) + "_" + std::to_string(v));
                        max_constraint_ub->SetCoefficient(Z, 1);
                        max_constraint_ub->SetCoefficient(p[{i, j, w, u}], -1);
                        max_constraint_ub->SetCoefficient(y[{i, j, v, w, u}], big_M);
                        
                        // Uniqueness constraint
                        max_constraint_unique->SetCoefficient(y[{i, j, v, w, u}], 1);
                    }

                    // Handling the conditional assignment of p_i_j(u,v) based on b_i_j(u,v)
                    // p_i_j(u,v) = Z if b_i_j(u,v) = 1, or 0 if b_i_j(u,v) = 0
                    // This is linearized through 4 constraints                    
                    // 1) p_i_j(u,v) <= Z
                    // 2) p_i_j(u,v) <= big_M*b_i_j(u,v)
                    // 3) p_i_j(u,v) >= Z - big_M*(1 - b_i_j(u,v))
                    // 4) p_i_j(u,v) >= 0

                    // Constraint: p_i_j(u,v) <= Z
                    MPConstraint* p_constraint1 = solver->MakeRowConstraint(-solver->infinity(), 0, "c7_p_constraint1_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                    p_constraint1->SetCoefficient(p[{i, j, u, v}], 1);
                    p_constraint1->SetCoefficient(Z, -1);

                    // Constraint: p_i_j(u,v) <= big_M*b_i_j(u,v)
                    MPConstraint* p_constraint2 = solver->MakeRowConstraint(-solver->infinity(), 0, "c7_p_constraint2_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                    p_constraint2->SetCoefficient(p[{i, j, u, v}], 1);
                    p_constraint2->SetCoefficient(b[{i, j, u, v}], -big_M);

                    // Constraint: p_i_j(u,v) >= Z - big_M*(1 - b_i_j(u,v))
                    MPConstraint* p_constraint3 = solver->MakeRowConstraint(-big_M, solver->infinity(), "c7_p_constraint3_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                    p_constraint3->SetCoefficient(p[{i, j, u, v}], 1);
                    p_constraint3->SetCoefficient(Z, -1);
                    p_constraint3->SetCoefficient(b[{i, j, u, v}], big_M);

                    // Constraint: p_i_j(u,v) >= 0
                    MPConstraint* p_constraint4 = solver->MakeRowConstraint(0, solver->infinity(), "c7_p_constraint4_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(u) + "_" + std::to_string(v));
                    p_constraint4->SetCoefficient(p[{i, j, u, v}], 1);

                }
            }
        }
        ++i;
    }
    // ----- C7 ends here -----

    // Define the objectives
    // O1: Minimize the maximum flow on any edge (Z = max_{(u,v)∈E} sum_{i,j} f_{i,j}(u,v))
    // Linearize: Z >= sum_{i,j} f_{i,j}(u,v) for all (u,v)∈E
    // Z - sum_{i,j} f_{i,j}(u,v) for all (u,v)∈E >= 0
    MPVariable* Z = solver->MakeNumVar(0.0, solver->infinity(), "O1");
    for (const auto& node : flowNetwork) { // Iterate over all nodes u in the network
        const int& u = node.first;
        for (const auto& v : node.second) { // Iterate over all edges (u,v) ∈ E
            MPConstraint* o1_edge = solver->MakeRowConstraint(0.0, solver->infinity(), "O1_constraint_" + std::to_string(u) + "_" + std::to_string(v));
            o1_edge->SetCoefficient(Z, 1);
            for (int i = 1; i <= M; ++i) {
                for (int j = 1; j <= K; ++j) {
                    o1_edge->SetCoefficient(f[{i, j, u, v}], -1);
                }
            }
        }
    }

    // O2: Minimize the total number of flit hops (O = sum_{(u,v)} sum_{i,j} b_{i,j}(u,v))
    MPVariable* O = solver->MakeNumVar(0.0, solver->infinity(), "O2");
    MPConstraint* o2 = solver->MakeRowConstraint(0.0, 0.0, "O2_constraint");
    o2->SetCoefficient(O, 1);
    for (const auto& node : flowNetwork) { // Iterate over all nodes u in the network
        const int& u = node.first;
        for (const auto& v : node.second) { // Iterate over all edges (u,v) ∈ E
            for (int i = 1; i <= M; ++i) {
                for (int j = 1; j <= K; ++j) {
                    o2->SetCoefficient(b[{i, j, u, v}], -1);  // O = sum_{i,j,u,v} b_{i,j}(u,v)
                }
            }
        }   
    }

    // -- Selecting Proper Coefficients for the objective functions
    /*
    The key idea behind selecting robust coefficients for the weighted sum objective is to ensure strict lexicographic 
    prioritization of the primary objective (minimizing MCL, Z) over the secondary objective (minimizing active channels, O), 
    while avoiding numerical instability. 
    By setting Z's coefficient to 1 and O's coefficient to 1/(Lmax+1), where Lmax is the worst-case total flow, we guarantee that any 
    improvement in Z dominates the objective value, as even the smallest reduction in Z  outweighs the largest possible change in O. 
    This ensures the solver first optimizes Z to its minimal possible value (Z∗) and only then minimizes O within the space of solutions where Z=Z∗.
    The secondary coefficient is scaled to be small enough to preserve this hierarchy but large enough to avoid floating-point precision issues, 
    effectively mimicking exact lexicographic optimization while using a standard weighted-sum solver.

    Another argument is: since O2 is only needed as tie-breaker (See paper for more), we might need to set O_Coff to just a small value (Careful not to cause numerical instability)
    like O_coff = 1e-6, maybe ask the Proferssor about it.
    */
    double L_max = 0.0;
    for (const auto& flow : flows) L_max += flow.second.second;
    double O_coeff = 1.0 / (L_max + 1.0);
    // -- 

    // Set the objective function
    MPObjective* objective = solver->MutableObjective();
    objective->SetCoefficient(Z, 1); // Primary objective: Minimize MCL
    objective->SetCoefficient(O, O_coeff); // Secondary objective: Minimize flit-hops
    objective->SetMinimization();

    
    solver->EnableOutput(); // enable solver's own log output
    // Solve the problem
    const MPSolver::ResultStatus result_status = solver->Solve();

    std::stringstream solverLog;
    milp_solver.getSolutionLog(result_status, objective, solverLog);

    writeLogToFile(solverLog, "app/output/milps/imr_fissionFusion_log.txt");
    writeConstraintsToFile(*solver, "app/output/milps/imr_fissionFusion_constraints.txt");
    WriteVariablesToFile(*solver, "app/output/milps/imr_fissionFusion_variables.txt");

    // Output the results
    // LOG(INFO) << "Maximum flow on any edge: " << Z->solution_value();
    // LOG(INFO) << "Total Flit Hops: " << O->solution_value();
}
}
