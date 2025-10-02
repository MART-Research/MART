#include <vector>
#include <map>
#include <set>
#include <sstream>
#include <stdlib.h>
#include "flowset.h"
#include "point.h"

#include "absl/flags/flag.h"
#include "absl/log/flags.h"
#include "ortools/base/init_google.h"
#include "ortools/base/logging.h"
#include "ortools/init/init.h"
#include "ortools/linear_solver/linear_solver.h"

using namespace std;

using namespace operations_research;

map<long, map<long, map<long, long long> > > f_fl_edges; // map<flow_id, map<node1, map<node2, f_var_ind> > >
map<long, map<long, map<long, long long> > > f_edge_flows; // map<node1, map<node2, map<flow_id, f_var_ind> > >

long long last_flow_index;

bool is_valid(point loc, point src, point dest, point &dims, bool is_mesh)
{
	bool ret = true;
	for(int d = 0; d < dims.n && dims[d] > 1; d++) {
		int t = src[d] - dest[d];
		int d_2 = dims[d] >> (1 - is_mesh);
		if(t > d_2 || -t > d_2) {
			t = d_2 - src[d];
			src[d] = d_2;
			dest[d] = (dest[d] + t + dims[d]) % dims[d];
			loc[d] = (loc[d] + t + dims[d]) % dims[d];
		}
		
		ret = ret && (loc[d] - src[d])*(loc[d] - dest[d]) <= 0;
	}

	return ret;
}


void or_write_var (MPSolver* solver, MPConstraint* constraint, std::map<int, MPVariable*>& x, 
	long flow, float wt, point &loc, point &neighbor, point &src, point &dest, point &dims, int sign, bool is_mesh) 
{
	loc.normalize(dims); neighbor.normalize(dims);
	
	if(is_valid(neighbor, src, dest, dims, is_mesh) && is_valid(loc, src, dest, dims, is_mesh)) 
	{
		
		map<long, map<long, map<long, long long> > >::iterator f_it;
		map<long, map<long, long long> >::iterator node1_it;
		map<long, long long>::iterator node2_it;
		
		if(	(f_it = f_fl_edges.find(flow)) == f_fl_edges.end() ||
			(node1_it = f_it->second.find(loc.get_code())) == f_it->second.end() ||
			(node2_it = node1_it->second.find(neighbor.get_code())) == node1_it->second.end()) {
			
			std::string flowVarName = "f_" + std::to_string(flow) + "_N" + loc.point_to_string() + "_N" + neighbor.point_to_string();
			x[last_flow_index] = solver->MakeNumVar(0.0, wt, flowVarName);
			
			constraint->SetCoefficient(x[last_flow_index], sign);
			f_fl_edges[flow][loc.get_code()][neighbor.get_code()] = last_flow_index;
			f_edge_flows[loc.get_code()][neighbor.get_code()][flow] = last_flow_index;
			last_flow_index++;
		} 
		else 
		{
			constraint->SetCoefficient(x[node2_it->second], sign);
		} 
	}
}


/*
	loc: is the intermediate node: it uniquely identifies the constraint, right?
*/
void or_write_flow_conservation_constraint (MPSolver* solver, std::map<int, MPVariable*>& x, 
	long flow, float wt, point &loc, point &src, point &dest, point &dims, point &delta, bool is_mesh) {
	MPConstraint *const constraint = solver->MakeRowConstraint(0, 0, "flowConservation_" + loc.point_to_string() + "_" + std::to_string(flow));
	for(int d = 0; d < delta.n && dims[d] > 1; d++) {
		point neighbor(loc);
		if(delta[d]) {
			// in edge
			neighbor[d] -= delta[d];
			or_write_var(solver, constraint, x, flow, wt, neighbor, loc, src, dest, dims, -1, is_mesh);
			// out edge
			neighbor[d] += 2 * delta[d]; // one to substitute for the above -delta and the other to add delta
			or_write_var(solver, constraint, x, flow, wt, loc, neighbor, src, dest, dims, 1, is_mesh);
		}
	}
}


void or_write_flow_satisfaction_constraint (MPSolver* solver, std::map<int, MPVariable*>& x, 
	long flow, float wt, point &src, point &dest, point &dims, point &delta, bool is_mesh) {
	MPConstraint *const constraint_src = solver->MakeRowConstraint(wt, wt, "flowSatisfaction_Src_" + std::to_string(flow) + "_" + src.point_to_string() + "_" + dest.point_to_string());
	MPConstraint *const constraint_dst = solver->MakeRowConstraint(wt, wt, "flowSatisfaction_Dst_" + std::to_string(flow) + "_" + src.point_to_string() + "_" + dest.point_to_string());
	for(int d = 0; d < delta.n && dims[d] > 1; d++) {
		point neighbor1(src);
		point neighbor2(dest);
		
		if(delta[d]) {
			// in edge
			neighbor1[d] += delta[d];
			or_write_var(solver, constraint_src, x, flow, wt, src, neighbor1, src, dest, dims, 1, is_mesh);
		
			// out edge
			neighbor2[d] -= delta[d]; // one to subst for the above -delta and the other to add delta
			or_write_var(solver, constraint_dst, x, flow, wt, neighbor2, dest, src, dest, dims, 1, is_mesh);
		}
	}

}

void or_write_Z_const(MPSolver* solver, std::map<int, MPVariable*>& x) {
	for(auto f_it1 = f_edge_flows.begin(); f_it1 != f_edge_flows.end(); f_it1++) {
		for(auto f_it2 = f_it1->second.begin(); f_it2 != f_it1->second.end(); f_it2++) {
			MPConstraint *const constraint_Z = solver->MakeRowConstraint(-solver->infinity(), 0, "constraint_Z" + std::to_string(f_it1->first) + "_" + std::to_string(f_it2->first));
			for(auto f_it3 = f_it2->second.begin(); f_it3 != f_it2->second.end(); f_it3++) {
				constraint_Z->SetCoefficient(x[f_it3->second], 1);
			}
			constraint_Z->SetCoefficient(x[0], -1);
		}
	}
	MPObjective *const objective = solver->MutableObjective();
	objective->SetCoefficient(x[0], 1);
	objective->SetMinimization();
}


void or_recurseDiff(MPSolver* solver, std::map<int, MPVariable*>& x, long flow, float wt, point &src, point &dest, point curr, point &diff, point &delta, point &dims, int dim, bool is_mesh)
{
	if(src != curr && dest != curr)
		or_write_flow_conservation_constraint(solver, x, flow, wt, curr, src, dest, dims, delta, is_mesh);
	
	for(int d = 0; d <= dim; d++) {
		if(diff[d] > 0) {
			curr[d] = (curr[d] + 1 + dims[d]) % dims[d];
			diff[d]--;

			or_recurseDiff(solver, x, flow, wt, src, dest, curr, diff, delta, dims, d, is_mesh);
			
			curr[d] = (curr[d] - 1 + dims[d]) % dims[d];
			diff[d]++;
		}
		else if(diff[d] < 0) {
			curr[d] = (curr[d] - 1 + dims[d]) % dims[d];
			diff[d]++;

			or_recurseDiff(solver, x, flow, wt, src, dest, curr, diff, delta, dims, d, is_mesh);

			curr[d] = (curr[d] + 1 + dims[d]) % dims[d];
			diff[d]--;
		}
	}
}


void or_build_LP(MPSolver* solver, std::map<int, MPVariable*>& x, FlowSet &fs, point &dims, map<long, point > mapping, bool is_mesh)
{
	x[0] = solver->MakeNumVar(0.0, solver->infinity(), "Z"); // define the variable of objective Z
	
	for(auto mapping_it = mapping.begin(); mapping_it != mapping.end(); mapping_it++) {
		for(int d = 0; d < dims.n && dims[d] > 1; d++) 
			mapping_it->second[d] = mapping_it->second[d] % dims[d];
	}

	map<long long, set<long> > edge_flows;

	long flow = 0;
	long const_num = 0;

	for(auto sdf_it = fs.flows.begin(); sdf_it != fs.flows.end(); sdf_it++) { // for each flow
		point src = mapping[sdf_it->first]; // source node
		for(auto dest_it = sdf_it->second.begin(); dest_it != sdf_it->second.end(); dest_it++) { // for each destination node of the flow
			point dest = mapping[dest_it->first];	// destination node
			float wt = dest_it->second; // flow demand
			point delta, diff;
			for(int d = 0; d < src.n && dims[d] > 1; d++) {
				if(src[d] == dest[d])
					delta[d] = 0;
				else if(src[d] < dest[d]) {
					if(is_mesh ||(dest[d] - src[d])<<1 <= dims[d]) {//if the distance is less than half the dimension, then go east, otherwise, wrap around
						delta[d] = 1;
						diff[d] = dest[d] - src[d];
					} else {
						delta[d] = -1;
						diff[d] = dest[d] - src[d] - dims[d];
					}
				} else {
					if(is_mesh || (src[d] - dest[d])<<1 <= dims[d]) {//if the distance is less than half the dimension, then go west, otherwise, wrap around
						delta[d] = -1;
						diff[d] = dest[d] - src[d];
					} else {
						delta[d] = 1;
						diff[d] = dims[d] - (src[d] - dest[d]);
					}
				}
			}
			or_write_flow_satisfaction_constraint(solver, x, flow, wt, src, dest, dims, delta, is_mesh);

			or_recurseDiff(solver, x, flow, wt, src, dest, src, diff, delta, dims, dims.n - 1, is_mesh);

			flow++;
		}
	}

	or_write_Z_const(solver, x);
	// MPObjective *const objective = solver->MutableObjective();
	// objective->SetCoefficient(x[0], 1);
	// objective->SetMinimization();
	// // solver->Solve();
}



float evaluate_mapping(FlowSet &fs, point &dims, map<long, point > &mapping, bool is_mesh) {
	f_fl_edges.clear();
	f_edge_flows.clear();
	last_flow_index = 1;
	
	std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("CBC"));
	map<int, MPVariable*> x;

	or_build_LP(solver.get(), x, fs, dims, mapping, is_mesh);

	// Solve the problem
    // LOG(INFO) << "Solving with " << solver->SolverVersion();
    const MPSolver::ResultStatus result_status = solver->Solve();
    // LOG(INFO) << "Status: " << result_status;   // Check that the problem has an optimal solution.
    if (result_status != MPSolver::OPTIMAL) {
        LOG(INFO) << "The problem does not have an optimal solution!";
        if (result_status == MPSolver::FEASIBLE)
            LOG(INFO) << "A potentially suboptimal solution was found";
        else
        LOG(WARNING) << "The solver could not solve the problem.";
        return -1;
    }

	float MCL = solver->Objective().Value();
	// LOG(INFO) << "Objective value: " << MCL;
	// LOG(INFO) << "Solution:";
	// for (const auto& var : x) {
		// LOG(INFO) << var.first << ": " << var.second->solution_value();
	// }
    return MCL;
}