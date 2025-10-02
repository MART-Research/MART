#include "../../utils/milps/orUtils.hpp"
#include "../../utils/milps/MARTUtility.hpp"
// #include "../utils/MILPs/CommunicationGraph.hpp"
#include "../../utils/milps/CDG.hpp"
#include "../../utils/milps/TurnModel.hpp"
#include "../utils/milps/MILPSolver.hpp"

#define SOURCE_ALIAS -1
#define DESTINATION_ALIAS -1

// Forward declaration of the MILP solver function
namespace operations_research {
    void solveMILP(
        MILPSolver& milp_solver,
        std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash> &flowNetwork, 
        std::unordered_map<std::pair<int, int>, int, pair_hash> &capacities,
        std::multimap<int, std::pair<int, double>> &flows, 
        std::unordered_map<std::pair<int, int>, int, pair_hash> &hop,
        int OBJECTIVE_NUM
    );
}


#ifndef BSOR_H
#define BSOR_H
class BSOR {
public:
    
    // Maybe this will need to be made more generic or better organized as we progress, but for now:
    // Assume BSOR takes a the aCDG, Da, and creates the flow network Ga with flows K
    BSOR
    (
        MILPSolver& solver,
        int num_nodes,
        std::unordered_map<std::pair<int, int>, int, pair_hash>& topo, 
        std::multimap<int, std::pair<int, double>> &communicationGraph, 
        int mainObjective, 
        bool is_2d_mesh, 
        int mesh_2d_rows, 
        int mesh_2d_cols
    ) 
    : 
    milpSolver(solver),
    numNodes(num_nodes),
    cdgInstance(extractKeys(topo)), 
    mappedTopology(topo), 
    commGraph(communicationGraph),
    objectiveNum(mainObjective), 
    is2dMesh(is_2d_mesh),
    numRows(mesh_2d_rows),
    numCols(mesh_2d_cols)
    {}

    // we are required to copy over the aCDG (Da) to a new graph in order to generate the flow network, Ga, on top of it, hence the pass by value
    // but actually we pass by reference since the passed flowNewtwork is the flowNetwork and we won't be using it (flowNetwork) again, hence we can
    // modify it directly to avoid extra memory overhead
    void generateFlowNetwork(std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash> &flowNetwork) { 
        // For all flows, store all flow sources and destinations, each in a separate, sorted list
        std::unordered_map<int, int>sources, destinations;
        for (const auto& flow: commGraph) {
            ++sources[flow.first]; // we increment, so we end up having how many times each node appeared as a source or destination
            ++destinations[flow.second.first];  // we aren't using this info now, as we only need to check it exists, but we might need it later Idk
        }
        // loop over a_CDG, for each edge-pair P1P2
        for (const auto& entry : flowNetwork) {
            if (sources[entry.first.first]) {   // if P1 is in sources
                flowNetwork[{SOURCE_ALIAS, entry.first.first}].push_back(entry.first); // add edge from P1_src, represented as {SOURCE_ALIAS, P1}, to P1P2
            }

            if (destinations[entry.first.second]) {     // if P2 is in destinations
                flowNetwork[entry.first].push_back({entry.first.second, DESTINATION_ALIAS});   // add edge from P1P2 to P2_dst, represented as {P2, DESTINATION_ALIAS}
            }
        }
        // note: for each Px, there is Px_src if it is the source in some pair P1P2, and there is Px_dst if it is the destination in some pair P1P2
    }


    void run_bsor() // it should be modified to return the set of optimal routing
    {
        // we first find the minimal route distances on the original graph, that is the hop vector
        std::vector<std::pair<int, int>> flowPairs;
        for (const auto& entry : commGraph) {
            int x = entry.first;
            int y = entry.second.first;
            flowPairs.push_back({x, y});
        }
        std::unordered_map<std::pair<int, int>, int, pair_hash> hop = shortestPathsForPairs(extractKeys(mappedTopology), flowPairs, numNodes);
        for (auto& entry : hop) {
            const auto& key = entry.first; // The pair (source, destination)
            auto& value = entry.second;    // The shortest path value
            if (value == INF) { // No path exists between source and destination
                std::cout << "NO solution: No path exists between the flow nodes: " << key.first << " -> " << key.second << "\n";
                return;
            } else {
                value = value + 1; // the minimal routes on the CDG graph are the shortest paths on the topology + 1
            }
        }
        

        // hop now contains the minimal hop lengths between the flows, needed for minimal routing
        // If non-minimal routing is needed, we can randomly or ad-hoc increase the hop lengths

        cdgInstance.createCDG();
        if (is2dMesh) // the default algorithm is described for 2D mesh, for which we use turn models to create acyclic CDG
        {
            // it should repeat for all turn models and cycle-breaking algorithms/methods. For now we are just trying for North Last turn model for simplicity
            NorthLastTurnModel northLastTurnModel;
            cdgInstance.setTurnModel(&northLastTurnModel);
            flowNetwork = cdgInstance.createACDG(numCols);  // numCols because our current running example is of size numCols, should be generalized later
            // up to here, flowNetwork is the acyclic CDG, Da
        }
        else // for other topologies, we assume the flow given topology is alread acyclic, so we just use the generated CDG (it is already an ACDG); no cycle-breaking needed
        {
            flowNetwork = cdgInstance.getCDG();
        }

        generateFlowNetwork(flowNetwork); // flowNetwork now represents the flow network Ga, we are just re-using the variable not to allocate new memory
        
        operations_research::solveMILP(milpSolver, flowNetwork, mappedTopology, commGraph, hop, objectiveNum);
    }

    
private:
    MILPSolver& milpSolver;
    int numNodes;
    CDG cdgInstance;
    std::multimap<int, std::pair<int, double>> commGraph;  // the flows, each flow is represented as {src, {dest, demand}}
    std::unordered_map<std::pair<int, int>, int, pair_hash> mappedTopology;
    int objectiveNum;
    bool is2dMesh;
    int numRows, numCols;
    std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash> flowNetwork;
};

#endif

// Define the MILP problem within the operations_research namespace
namespace operations_research 
{
    void solveMILP(
        MILPSolver& milp_solver,
        std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash> &flowNetwork, 
        std::unordered_map<std::pair<int, int>, int, pair_hash> &capacities,
        std::multimap<int, std::pair<int, double>> &flows, 
        std::unordered_map<std::pair<int, int>, int, pair_hash> &hop,
        int OBJECTIVE_NUM
    )
    {   
        // Retrieve the solver
        MPSolver* solver = milp_solver.get();

        std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash> reverseFlowNetwork;
        for (const auto& vertex : flowNetwork) {
            std::pair<int, int> src = vertex.first;
            const auto& vec = vertex.second;
            for (const auto& dst : vec)
                reverseFlowNetwork[dst].emplace_back(src);
        }
    
        // auxiliary variable that we will use in looping over flows
        int i = 1;
        
        // Define the Variables
        std::map<std::tuple<int, int, int, int, int>, operations_research::MPVariable*> f; // fi(u,v) : flow variable for each edge (u,v) and each flow i
        std::map<std::tuple<int, int, int, int, int>, operations_research::MPVariable*> b; // bi(u,v) : boolean variable indicating if edge (u,v) is used by flow i  
        i = 1;
        std::vector<std::tuple<int, int, int, int>> TraversibleEdges = getAllEdges(flowNetwork);
        for (const auto& flow : flows) {
            const int si = flow.first, ti = flow.second.first;
            const double di = flow.second.second;
            for (const auto& edge : TraversibleEdges) {
                const int &p1_1 = std::get<0>(edge), &p1_2 = std::get<1>(edge), &p2_1 = std::get<2>(edge), &p2_2 = std::get<3>(edge);
                std::string flowVarName = "f_" + std::to_string(i) + "_" + std::to_string(p1_1) + "_" + std::to_string(p1_2)
                + "_" + std::to_string(p2_1) + "_" + std::to_string(p2_2);
                f[{i, p1_1, p1_2, p2_1, p2_2}] = solver->MakeIntVar(0.0, di, flowVarName);
                flowVarName[0] = 'b';
                b[{i, p1_1, p1_2, p2_1, p2_2}] = solver->MakeBoolVar(flowVarName);
            }
            ++i;
        }
    
        // Define the constraints
    
        // Capacity Constraint : For any node, except si and ti, the sum of fi into it is less than the node capacity
        // Note that the capacity constraints are not applicable in case of OBJECTIVE_NUM 3 (Minimization of MCL)
        if (OBJECTIVE_NUM == 1 || OBJECTIVE_NUM == 2) {
          for (const auto& v: reverseFlowNetwork) {
            const int &v_src = v.first.first, &v_dst = v.first.second;
            if (v_dst == SOURCE_ALIAS || v_src == SOURCE_ALIAS) continue;   // v = si or v = ti (shouldn't need to check for v=si if reverseFlowNetwork contains vertices that are pointed to by at least one other vertex; a converse case is to contain an si, all of which are not pointed to by any other vertex and hence the need to check for v=si)
            std::string constraintName = "CAP_" + std::to_string(v_src) + "_" + std::to_string(v_dst);
            MPConstraint *const capacity_constraint = solver->MakeRowConstraint(0.0, capacities[{v_src, v_dst}], constraintName);
            const auto& vec = v.second;   // vector of all vertices that point to vertex v
            for (const auto& u: vec) {
                int i = 1;
                for (const auto& flow : flows) {
                    capacity_constraint->SetCoefficient(f[{i, u.first, u.second, v_src, v_dst}], 1);
                    ++i;
                }
            }
          }
        }
    
        // Flow Conservation Constraints
        // a] For any node, except si and ti, the sum of fi into the node equals the sum of fi out of it
        i = 1;
        for (const auto& flow: flows) {
            for (const auto& u : flowNetwork) {
                const int &u_src = u.first.first, &u_dst = u.first.second;
                if (u_src == SOURCE_ALIAS || u_dst == SOURCE_ALIAS) continue;   // u = si or v = ti
                std::string constraintName = "FCa_" + std::to_string(i) + "_" + std::to_string(u_src) + "_" + std::to_string(u_dst);
                MPConstraint *const flow_conservation_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);
                const auto& vec = u.second;
                for (const auto& w : vec)
                    flow_conservation_constraint->SetCoefficient(f[{i, u_src, u_dst, w.first, w.second}], 1);
                const auto& rev_vec = reverseFlowNetwork[{u_src, u_dst}];
                for (const auto& w: rev_vec)
                    flow_conservation_constraint->SetCoefficient(f[{i, w.first, w.second, u_src, u_dst}], -1);
            }
            ++i;
        }
    
        // b] For any flow, the sum of flow weight out of the source node equals the sum of the flow weight into the destination node,
        //    equal some constant gi, which represents the actual throughput for flow i
        i = 1;
        for (const auto& flow: flows) {
            int si = flow.first, ti = flow.second.first;
            std::string constraintName = "FCb_" + std::to_string(i);
            MPConstraint *const flow_conservation_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);
            const auto& vec = flowNetwork[{SOURCE_ALIAS, si}];
            for (const auto& w : vec)
                flow_conservation_constraint->SetCoefficient(f[{i, SOURCE_ALIAS, si, w.first, w.second}], 1);
            const auto& rev_vec = reverseFlowNetwork[{ti, DESTINATION_ALIAS}];
            for (const auto& w : rev_vec)
                flow_conservation_constraint->SetCoefficient(f[{i, w.first, w.second, ti, DESTINATION_ALIAS}], -1);
            ++i;
        }
    
        // Unsplittable Flow
        // a] ∀i,∀(u,v)∈E fi(u,v) ≤ bi(u,v)·di, where bi(u,v)=1 indicates that flow i uses the edge (u,v) for all the flow demand di
        i = 1;
        for (const auto& flow: flows) {
            double demand = flow.second.second;
            for (const auto& u : flowNetwork) {
                const int &u_src = u.first.first, &u_dst = u.first.second;
                const auto& vec = u.second;
                for (const auto& v : vec) {
                    int v_src = v.first, v_dst = v.second;
                    std::string constraintName = "USFa_" + std::to_string(i) + "_" + std::to_string(u_src) + "_" + std::to_string(u_dst)
                    + "_" + std::to_string(v_src) + "_" + std::to_string(v_dst);
                    MPConstraint *const unsplittable_flow_constraint = solver->MakeRowConstraint(0.0, 0.0, constraintName);
                    unsplittable_flow_constraint->SetCoefficient(f[{i, u_src, u_dst, v_src, v_dst}], 1);
                    unsplittable_flow_constraint->SetCoefficient(b[{i, u_src, u_dst, v_src, v_dst}], -1 * demand);
                }
            }
            ++i;
        }
    
        // b] ∀i, ∀u summation over((u,v) ∈ E) : bi(u,v) ≤ 1, where bi(u,v) is a binary variable
        //    For all flows, for any node, the link between it and any other node can have at most 1 flow
        i = 1;  
        for (const auto& flow: flows) {
            for (const auto& u : flowNetwork) {
                const int &u_src = u.first.first, &u_dst = u.first.second;
                std::string constraintName = "USFb_" + std::to_string(i) + "_" + std::to_string(u_src) + "_" + std::to_string(u_dst);
                MPConstraint *const unsplittable_flow_constraint = solver->MakeRowConstraint(0.0, 1.0, constraintName);
                const auto& vec = u.second;
                for (const auto& v: vec)
                    unsplittable_flow_constraint->SetCoefficient(b[{i, u_src, u_dst, v.first, v.second}], 1);
            }
            ++i;
        }
    
        // Hop : ∀i summation over((u,v)∈E) bi(u,v) ≤ hopi.
        i = 1;
        for (const auto& flow: flows) {
            int si = flow.first, ti = flow.second.first;
            std::string constraintName = "HopCnt_" + std::to_string(i);
            MPConstraint *const Hop_count_constraint = solver->MakeRowConstraint(0.0, hop[{si, ti}], constraintName);
            for (const auto& u : flowNetwork) {
                const int &u_src = u.first.first, &u_dst = u.first.second;
                const auto& vec = u.second;
                for (const auto& v: vec)
                    Hop_count_constraint->SetCoefficient(b[{i, u_src, u_dst, v.first, v.second}], 1);
            }
            ++i;
        }
    
    
        // Possible Objectives / Cost Functions
        // 1] Total Throughput maximization
    
        // 2] maximize the minimal fraction of the flow of each commodity to its demand satisfaction (max-min fairness)
        // In most applications, flows are correlated, i.e., throttling one flow will affect the throughput demand of another
        // This objective is originally formulated as : maximize min(gi/di, 1<=i<=k) . This contains a min operation, which is a non-linear operation.
        // To linearize this objective, assume some auxiliary variable Z = min(gi/di, 1<=i<=k)
        // The equivalent linear formulation is: maximize Z, subject to Z <= gi/di, 1<=i<=k --> gi - di * Z >= 0
    
        // 3] minimize the maximum channel load (MCL)
        // This can be done regardless of network capacity, and only knowing the relative demands of flows.
        // This objective is originally formulated as : minimize max(h(v), ∀v ∈ V) . This contains a max operation, which is a non-linear operation.
        // To linearize this objective, assume some auxiliary variable Z = max(h(v), ∀v ∈ V)
        // The equivalent linear formulation is: minimize Z, subject to Z >= h(v), ∀v ∈ V --> h(v) - Z <= 0
        // The capacity constraints are dropped; instead we enforce the extra constraint gi = di, at both the source and destination nodes
        // If this is not explicitly enforced, then we end up with the trivial solution of all fi's = 0, which is not acceptable, of course
        
        MPObjective* const objective = solver->MutableObjective();
    
        if (OBJECTIVE_NUM == 1) {  // maximize total throughput
            i = 1;
            for (const auto& flow: flows) {
                int si = flow.first;  // this could objective could be set at si or ti (it won't matter because constraint Flow Conservation b] ensures gi(src) = gi(dst))
                const auto& vec = flowNetwork[{SOURCE_ALIAS, si}];
                for (const auto& w: vec)
                    objective->SetCoefficient(f[{i, SOURCE_ALIAS, si, w.first, w.second}], 1);
                ++i;
            }
            objective->SetMaximization();
        } else if (OBJECTIVE_NUM == 2) { // maximize min fairness
            // Linearize the objective
            operations_research::MPVariable* const Z = solver->MakeNumVar(0.0, solver->infinity(), "O2");
            i = 1;
            for (const auto& flow: flows) {
                int si = flow.first;
                double di = flow.second.second;
                std::string constraintName = "MinFracFlow_" + std::to_string(i);
                MPConstraint *const minimal_fractional_flow_constraint = solver->MakeRowConstraint(0.0, solver->infinity(), constraintName);
                minimal_fractional_flow_constraint->SetCoefficient(Z, -1 * di);
                const auto& vec = flowNetwork[{SOURCE_ALIAS, si}];
                for (const auto& w: vec)
                    minimal_fractional_flow_constraint->SetCoefficient(f[{i, SOURCE_ALIAS, si, w.first, w.second}], 1);
                ++i;
            }
            objective->SetCoefficient(Z, 1);
            objective->SetMaximization();
        } else {  // OBJECTIVE_NUM = 3 -> minimze MCL
            // Add gi = di constraint
            i = 1;
            for (const auto& flow: flows) {
                int si = flow.first, ti = flow.second.first;
                double di = flow.second.second;
                std::string constraintName = "TP_src_" + std::to_string(i);
                MPConstraint *const src_throughput_constraint = solver ->MakeRowConstraint(di, di, constraintName);
                constraintName = "TP_dst_" + std::to_string(i);
                MPConstraint *const dst_throughput_constraint = solver ->MakeRowConstraint(di, di, constraintName);
                const auto& vec_src = flowNetwork[{SOURCE_ALIAS, si}];
                for (const auto& w: vec_src) 
                    src_throughput_constraint->SetCoefficient(f[{i, SOURCE_ALIAS, si, w.first, w.second}], 1);
                const auto& vec_dst = reverseFlowNetwork[{ti, DESTINATION_ALIAS}];
                for (const auto& w: vec_dst)
                    dst_throughput_constraint->SetCoefficient(f[{i, w.first, w.second, ti, DESTINATION_ALIAS}], 1);
                ++i;
            }
            // Linearize the objective
            // minimize Z = max(v ∈ V) h(v)
            // Linearize: Z >= h(v) for all v ∈ V
            // 0 >= -Z + h(v) for all v ∈ V
            operations_research::MPVariable* const Z = solver->MakeNumVar(0.0, solver->infinity(), "O3");
            for (const auto& v : reverseFlowNetwork) {
                int v_src = v.first.first, v_dst = v.first.second;
                if (v_dst == SOURCE_ALIAS) continue;  // v = si is already excluded as we are looping on reverseFlowNetwork, it remains to exclude v = ti
                std::string constraintName = "MCL_" + std::to_string(v_src) + "_" + std::to_string(v_dst);
                MPConstraint* const link_constraint = solver->MakeRowConstraint(-solver->infinity(), 0.0, constraintName);
                link_constraint->SetCoefficient(Z, -1);
                const auto& vec = v.second;
                for (const auto& u: vec) {
                    int i = 1;
                    for (const auto& flow : flows) {    // K flows
                        link_constraint->SetCoefficient(f[{i, u.first, u.second, v_src, v_dst}], 1);
                        ++i;
                    }
                }
            }
            objective->SetCoefficient(Z, 1);
            objective->SetMinimization();
        }
    
        solver->EnableOutput(); // enable solver's own log output
        // Solve the problem
        const MPSolver::ResultStatus result_status = solver->Solve();

        std::stringstream solverLog;
        milp_solver.getSolutionLog(result_status, objective, solverLog);
        
        writeLogToFile(solverLog, "app/output/milps/bsor_milp_output.txt");
        WriteVariablesToFile(*solver, "app/output/milps/bsor_milp_output.txt", "append");
    }

}
