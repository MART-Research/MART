#ifndef CUBEP_OR_H
#define CUBEP_OR_H

#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "point.h"
#include "nodemappings.h"
#include "flowset.h"

#include "ortools/base/logging.h"
#include "ortools/linear_solver/linear_solver.h"

using namespace std;
using namespace operations_research;

#define HYPER_GAP_THRESH 0.0001

class HyperCubePartition {
public:
  static void partitionHyperCube(FlowSet& fs, int ndims, NodeMappings& nm) {
    if (ndims > 1) {
      FlowSet fs1, fs2;
      point   dims2(nm.dims);
      dims2[ndims - 1] = 1;                // collapse last dim

      NodeMappings m1, m2;
      m1.setDims(dims2);
      m2.setDims(dims2);

      partitionILP(fs, fs1, fs2, ndims);

      partitionHyperCube(fs1, ndims - 1, m1);
      partitionHyperCube(fs2, ndims - 1, m2);

      nm.rotateAndMergeMapping(m1, m2, fs, 1);
    } else {
      nm.triviallyMap(fs.nodes);
    }
  }
  static void buildILP(MPSolver* solver, std::vector<MPVariable*>& var, FlowSet& fs, int ndims) {

    long var_count = 1;

    int n_inter_links = (ndims > 2) ? 1 << (ndims - 1) : 2;
    int n_intra_links = (ndims > 2) ? 1 << (ndims - 1) : 1;
    double scale_inter = 1.0 / n_inter_links;
    double scale_intra = 1.0 / n_intra_links;

    
    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), "Z"));
    solver->MutableObjective()->SetCoefficient(var[0], 1.0);
    solver->MutableObjective()->SetMinimization();

    
    map<long,long> node_rename;
    long rename_counter = 1;

    MPConstraint* sum_g_upper = solver->MakeRowConstraint(-solver->infinity(), fs.nodes.size() / 2);
    MPConstraint* sum_g_lower = solver->MakeRowConstraint(fs.nodes.size() / 2, solver->infinity());

    for (long node : fs.nodes) {
      node_rename[node] = rename_counter;

      stringstream ss0, ss1;
      ss0 << "G_" << rename_counter << "_0";
      ss1 << "G_" << rename_counter << "_1";

      var.push_back(solver->MakeBoolVar(ss0.str())); // index = var_count
      var.push_back(solver->MakeBoolVar(ss1.str())); // index = var_count+1

      
      MPConstraint* eq1 = solver->MakeRowConstraint(1.0, 1.0);
      eq1->SetCoefficient(var[var_count],     1.0);
      eq1->SetCoefficient(var[var_count + 1], 1.0);

      sum_g_upper->SetCoefficient(var[var_count], 1.0);
      sum_g_lower->SetCoefficient(var[var_count], 1.0);

      var_count += 2;
      ++rename_counter;
    }

    
    std::vector<int> part1_idx;
    std::vector<int> part2_idx;

    for (auto& [src, dst_map] : fs.flows) {
      long src_r = node_rename[src];

      for (auto& [dst, wt] : dst_map) {
        long dst_r = node_rename[dst];

        
		    std::ostringstream nm1;
		    nm1 << "F_" << src << '_' << dst << "_p1";
		    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), nm1.str()));
        int v1 = var_count;

        MPConstraint* c1 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
        c1->SetCoefficient(var[v1], 1.0);
        c1->SetCoefficient(var[2 * src_r - 1], -wt);

        MPConstraint* c2 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
        c2->SetCoefficient(var[v1], 1.0);
        c2->SetCoefficient(var[2 * dst_r - 1], -wt);

        MPConstraint* c3 = solver->MakeRowConstraint(-solver->infinity(), wt);
        c3->SetCoefficient(var[2 * src_r - 1], wt);
        c3->SetCoefficient(var[2 * dst_r - 1], wt);
        c3->SetCoefficient(var[v1], -1.0);

        part1_idx.push_back(v1);
        ++var_count;

        
		    std::ostringstream nm2;
		    nm2 << "F_" << src << '_' << dst << "_p2";
		    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), nm2.str()));
        int v2 = var_count;

        MPConstraint* c4 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
        c4->SetCoefficient(var[v2], 1.0);
        c4->SetCoefficient(var[2 * src_r], -wt);

        MPConstraint* c5 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
        c5->SetCoefficient(var[v2], 1.0);
        c5->SetCoefficient(var[2 * dst_r], -wt);

        MPConstraint* c6 = solver->MakeRowConstraint(-solver->infinity(), wt);
        c6->SetCoefficient(var[2 * src_r], wt);
        c6->SetCoefficient(var[2 * dst_r], wt);
        c6->SetCoefficient(var[v2], -1.0);

        part2_idx.push_back(v2);
        ++var_count;
      }
    }

    
    int p1_sum_var = var_count;
    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), "P1_sum"));

    MPConstraint* p1_sum_def = solver->MakeRowConstraint(0.0, 0.0);
    p1_sum_def->SetCoefficient(var[p1_sum_var], -1.0);
    for (int idx : part1_idx) p1_sum_def->SetCoefficient(var[idx], 1.0);
    ++var_count;

    int p2_sum_var = var_count;
    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), "P2_sum"));

    MPConstraint* p2_sum_def = solver->MakeRowConstraint(0.0, 0.0);
    p2_sum_def->SetCoefficient(var[p2_sum_var], -1.0);
    for (int idx : part2_idx) p2_sum_def->SetCoefficient(var[idx], 1.0);
    ++var_count;

    
    MPConstraint* c7 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
    c7->SetCoefficient(var[p1_sum_var], scale_inter);
    c7->SetCoefficient(var[0], -1.0);

    MPConstraint* c8 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
    c8->SetCoefficient(var[p2_sum_var], scale_inter);
    c8->SetCoefficient(var[0], -1.0);

    MPConstraint* c9 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
    c9->SetCoefficient(var[p1_sum_var], -scale_intra);
    c9->SetCoefficient(var[p2_sum_var], -scale_intra);
    c9->SetCoefficient(var[0], -1.0);
    c9->SetBounds(-solver->infinity(), -scale_intra * fs.total_wt);
  }

  static void partitionILP(FlowSet& fs, FlowSet& fs1, FlowSet& fs2, int ndims) {

    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("CBC"));
    if (!solver) { cerr << "Could not create CBC solver\n"; exit(1); }

    vector<MPVariable*> var;
    buildILP(solver.get(), var, fs, ndims);

    /* same gap as IloCplex::EpGap */
    // solver->SetSolverSpecificParametersAsString("ratioGap=" + std::to_string(HYPER_GAP_THRESH));

    solver->Solve();

    /* extract solution and split nodes */
    float MCL = static_cast<float>(var[0]->solution_value());

    int i = 1;
    for (long node : fs.nodes) {
      if (var[i << 1]->solution_value() > 0.1)
        fs2.nodes.insert(node);
      else
        fs1.nodes.insert(node);
      ++i;
    }
    fs.splitFlows(fs1.nodes, fs1, fs2);
  }
};

#endif  /* CUBEP_OR_H */