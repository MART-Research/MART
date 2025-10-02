#ifndef DRAGONFLYP_OR_H
#define DRAGONFLYP_OR_H

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

#define DFLY_GAP_THRESH 0.0001


struct DflyAddr {
  int g, r, p;
  static DflyAddr decode(long node, int routers_per_group, int ports_per_router)
  {
    DflyAddr a;
    a.p = node % ports_per_router;
    node /= ports_per_router;
    a.r = node % routers_per_group;
    a.g = node / routers_per_group;
    return a;
  }
};

class DragonflyPartition {
public:
  static void partition(FlowSet&     fs, int groups, int routers_per_group, int ports_per_router, NodeMappings& nm)
  {
    if (groups > 1)
      recurse_level(fs, groups, routers_per_group, ports_per_router, Level::GROUP, nm);
    else if (routers_per_group > 1)
      recurse_level(fs, groups, routers_per_group, ports_per_router, Level::ROUTER, nm);
    else if (ports_per_router > 1)
      recurse_level(fs, groups, routers_per_group, ports_per_router, Level::PORT, nm);
    else
      nm.triviallyMap(fs.nodes);
  }

private:
  enum class Level { GROUP, ROUTER, PORT };

  static void recurse_level(FlowSet& fs, int& groups, int& routers_per_group, int& ports_per_router, Level    level, NodeMappings& nm)
  {
    FlowSet fs1, fs2;

    
    for (long node : fs.nodes) {
      DflyAddr a = DflyAddr::decode(node, routers_per_group, ports_per_router);
      bool to_left = false;
      switch (level) {
        case Level::GROUP:  to_left = (a.g <  groups / 2); break;
        case Level::ROUTER: to_left = (a.r <  routers_per_group / 2); break;
        case Level::PORT:   to_left = (a.p <  ports_per_router / 2);  break;
      }
      (to_left ? fs1.nodes : fs2.nodes).insert(node);
    }
    fs.splitFlows(fs1.nodes, fs1, fs2);

    
    apply_ILP_balance(fs, fs1, fs2, groups, routers_per_group, ports_per_router, level);

    // recurse deeper
    NodeMappings m1, m2;
    m1.setDims(nm.dims);
    m2.setDims(nm.dims);

    int ng = groups,
        nr = routers_per_group,
        np = ports_per_router;
    switch (level) {
      case Level::GROUP:  ng /= 2; break;
      case Level::ROUTER: nr /= 2; break;
      case Level::PORT:   np /= 2; break;
    }

    if (!fs1.nodes.empty())
      partition(fs1, ng, nr, np, m1);
    if (!fs2.nodes.empty())
      partition(fs2, ng, nr, np, m2);

    nm.rotateAndMergeMapping(m1, m2, fs, 1);
  }

  static void apply_ILP_balance(FlowSet& fs, FlowSet& fs1, FlowSet& fs2, int groups, int routers_per_group, int ports_per_router, Level level)
  {
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("CBC"));
    if (!solver) { cerr << "CBC solver creation failed\n"; exit(1); }

    vector<MPVariable*> var;
    buildILP(solver.get(), var, fs, groups, routers_per_group, ports_per_router, level);

    solver->Solve();
    
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


  static void buildILP(MPSolver* solver, vector<MPVariable*>& var, FlowSet& fs, int groups, int routers_per_group, int ports_per_router, Level level)
  {
    long var_count = 1;

    
    int n_inter_links = 1;
    int n_intra_links = 1;

    switch (level) {
      case Level::GROUP:
        n_inter_links = 1;
        n_intra_links = routers_per_group * ports_per_router - 1;
        break;
      case Level::ROUTER:
        n_inter_links = 1;
        n_intra_links = ports_per_router - 1;
        break;
      case Level::PORT:
        n_inter_links = 1;
        n_intra_links = 0;
        break;
    }
    double scale_inter = 1.0 / n_inter_links;
    double scale_intra = n_intra_links ? 1.0 / n_intra_links : 1.0;

    
    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), "Z"));
    solver->MutableObjective()->SetCoefficient(var[0], 1.0);
    solver->MutableObjective()->SetMinimization();

    map<long,long> node_rename;  long rename_counter = 1;
    MPConstraint* sum_g_upper =
        solver->MakeRowConstraint(-solver->infinity(), fs.nodes.size() / 2);
    MPConstraint* sum_g_lower =
        solver->MakeRowConstraint(fs.nodes.size() / 2, solver->infinity());

    for (long node : fs.nodes) {
      node_rename[node] = rename_counter;
      stringstream s0, s1;
      s0 << "G_" << rename_counter << "_0";
      s1 << "G_" << rename_counter << "_1";
      var.push_back(solver->MakeBoolVar(s0.str()));
      var.push_back(solver->MakeBoolVar(s1.str()));
      MPConstraint* eq = solver->MakeRowConstraint(1.0, 1.0);
      eq->SetCoefficient(var[var_count],     1.0);
      eq->SetCoefficient(var[var_count + 1], 1.0);
      sum_g_upper->SetCoefficient(var[var_count], 1.0);
      sum_g_lower->SetCoefficient(var[var_count], 1.0);
      var_count += 2; ++rename_counter;
    }

    vector<int> part1_idx, part2_idx;
    for (auto& [src, dst_map] : fs.flows) {
      long src_r = node_rename[src];
      for (auto& [dst, wt] : dst_map) {
        long dst_r = node_rename[dst];

        
        stringstream n1; n1 << "F_" << src << '_' << dst << "_p1";
        var.push_back(solver->MakeNumVar(0.0, solver->infinity(), n1.str()));
        int v1 = var_count;
        MPConstraint* c1 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
        c1->SetCoefficient(var[v1], 1.0);
        c1->SetCoefficient(var[2*src_r-1], -wt);
        MPConstraint* c2 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
        c2->SetCoefficient(var[v1], 1.0);
        c2->SetCoefficient(var[2*dst_r-1], -wt);
        MPConstraint* c3 = solver->MakeRowConstraint(-solver->infinity(), wt);
        c3->SetCoefficient(var[2*src_r-1], wt);
        c3->SetCoefficient(var[2*dst_r-1], wt);
        c3->SetCoefficient(var[v1], -1.0);
        part1_idx.push_back(v1);
        ++var_count;

        
        stringstream n2; n2 << "F_" << src << '_' << dst << "_p2";
        var.push_back(solver->MakeNumVar(0.0, solver->infinity(), n2.str()));
        int v2 = var_count;
        MPConstraint* c4 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
        c4->SetCoefficient(var[v2], 1.0);
        c4->SetCoefficient(var[2*src_r], -wt);
        MPConstraint* c5 = solver->MakeRowConstraint(-solver->infinity(), 0.0);
        c5->SetCoefficient(var[v2], 1.0);
        c5->SetCoefficient(var[2*dst_r], -wt);
        MPConstraint* c6 = solver->MakeRowConstraint(-solver->infinity(), wt);
        c6->SetCoefficient(var[2*src_r], wt);
        c6->SetCoefficient(var[2*dst_r], wt);
        c6->SetCoefficient(var[v2], -1.0);
        part2_idx.push_back(v2);
        ++var_count;
      }
    }

    int p1_sum_var = var_count;
    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), "P1_sum"));
    MPConstraint* p1_def = solver->MakeRowConstraint(0.0, 0.0);
    p1_def->SetCoefficient(var[p1_sum_var], -1.0);
    for (int i : part1_idx) p1_def->SetCoefficient(var[i], 1.0);
    ++var_count;

    int p2_sum_var = var_count;
    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), "P2_sum"));
    MPConstraint* p2_def = solver->MakeRowConstraint(0.0, 0.0);
    p2_def->SetCoefficient(var[p2_sum_var], -1.0);
    for (int i : part2_idx) p2_def->SetCoefficient(var[i], 1.0);
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
};

#endif /* DRAGONFLYP_OR_H */
