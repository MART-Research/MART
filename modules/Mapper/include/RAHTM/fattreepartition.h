#ifndef FATTREEP_OR_H
#define FATTREEP_OR_H

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

#define FAT_GAP_THRESH 0.0001

class FatTreePartition {
public:
  static void partitionFatTree(FlowSet& fs, int fanout, int level, NodeMappings& nm) {

    
    if (fanout == 1) {
      nm.triviallyMap(fs.nodes);
      return;
    }

    
    FlowSet fsUp, fsDown;
    partitionILP(fs, fsUp, fsDown, fanout);

    point childDims(nm.dims);
    childDims[level] = childDims[level] / 2;

    NodeMappings nmUp, nmDown;
    nmUp.setDims(childDims);
    nmDown.setDims(childDims);

    partitionFatTree(fsUp,   fanout / 2, level-1, nmUp);
    partitionFatTree(fsDown, fanout / 2, level-1, nmDown);

    
    nm.rotateAndMergeMapping(nmDown, nmUp, fs, 1);
  }

private:
  
  static void buildILP(MPSolver* solver, std::vector<MPVariable*>& var, FlowSet& fs, int fanout) {

  
    const int n_up   = fanout / 2;
    const int n_down = fanout / 2;

    const double scaleInter = 1.0 / n_up;
    const double scaleIntra = 1.0 / n_down;

    long vc = 1;

  
    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), "Z"));
    solver->MutableObjective()->SetCoefficient(var[0], 1.0);
    solver->MutableObjective()->SetMinimization();

  
    map<long,long> rename;
    long id = 1;
    MPConstraint* sum_up = solver->MakeRowConstraint(fanout/2, fanout/2);
    for (long node : fs.nodes) {
      rename[node] = id;
      string g0 = "G_"+to_string(id)+"_0";
      string g1 = "G_"+to_string(id)+"_1";
      var.push_back(solver->MakeBoolVar(g0));
      var.push_back(solver->MakeBoolVar(g1));

      MPConstraint* eq = solver->MakeRowConstraint(1,1);
      eq->SetCoefficient(var[vc],   1);
      eq->SetCoefficient(var[vc+1], 1);

      sum_up->SetCoefficient(var[vc], 1);
      vc += 2; id++;
    }

    // Flow variables + constraints
    std::vector<int> upIdx, dnIdx;

    for (auto& [src, dstMap] : fs.flows) {
      long rs = rename[src];
      for (auto& [dst, w] : dstMap) {
        long rd = rename[dst];

        string n1 = "F_"+to_string(src)+"_"+to_string(dst)+"_up";
        var.push_back(solver->MakeNumVar(0.0, solver->infinity(), n1));
        int fup = vc++;

        // up-link constraints
        MPConstraint* c1 = solver->MakeRowConstraint(-solver->infinity(),0);
        c1->SetCoefficient(var[fup], 1);
        c1->SetCoefficient(var[2*rs-1], -w);
        MPConstraint* c2 = solver->MakeRowConstraint(-solver->infinity(),0);
        c2->SetCoefficient(var[fup], 1);
        c2->SetCoefficient(var[2*rd-1], -w);
        MPConstraint* cb = solver->MakeRowConstraint(-solver->infinity(), w);
        cb->SetCoefficient(var[2*rs-1], w);
        cb->SetCoefficient(var[2*rd-1], w);
        cb->SetCoefficient(var[fup], -1);

        upIdx.push_back(fup);

        string n2 = "F_"+to_string(src)+"_"+to_string(dst)+"_dn";
        var.push_back(solver->MakeNumVar(0.0, solver->infinity(), n2));
        int fdn = vc++;

        MPConstraint* d1 = solver->MakeRowConstraint(-solver->infinity(),0);
        d1->SetCoefficient(var[fdn], 1);
        d1->SetCoefficient(var[2*rs], -w);
        MPConstraint* d2 = solver->MakeRowConstraint(-solver->infinity(),0);
        d2->SetCoefficient(var[fdn], 1);
        d2->SetCoefficient(var[2*rd], -w);
        MPConstraint* db = solver->MakeRowConstraint(-solver->infinity(), w);
        db->SetCoefficient(var[2*rs], w);
        db->SetCoefficient(var[2*rd], w);
        db->SetCoefficient(var[fdn], -1);

        dnIdx.push_back(fdn);
      }
    }

    // Sum variables P_up / P_dn
    int pup = vc;
    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), "P_up"));
    MPConstraint* defUp = solver->MakeRowConstraint(0,0);
    defUp->SetCoefficient(var[pup], -1);
    for(int i : upIdx) defUp->SetCoefficient(var[i], 1);
    vc++;

    int pdn = vc;
    var.push_back(solver->MakeNumVar(0.0, solver->infinity(), "P_dn"));
    MPConstraint* defDn = solver->MakeRowConstraint(0,0);
    defDn->SetCoefficient(var[pdn], -1);
    for(int i : dnIdx) defDn->SetCoefficient(var[i], 1);
    vc++;

    // Z ≥ scaled loads
    MPConstraint* z1 = solver->MakeRowConstraint(-solver->infinity(),0);
    z1->SetCoefficient(var[pup], scaleInter);
    z1->SetCoefficient(var[0],  -1);

    MPConstraint* z2 = solver->MakeRowConstraint(-solver->infinity(),0);
    z2->SetCoefficient(var[pdn], scaleInter);
    z2->SetCoefficient(var[0],  -1);

    MPConstraint* z3 = solver->MakeRowConstraint(-solver->infinity(), -scaleIntra*fs.total_wt);
    z3->SetCoefficient(var[pup], -scaleIntra);
    z3->SetCoefficient(var[pdn], -scaleIntra);
    z3->SetCoefficient(var[0],   -1);
  }

  
  static void partitionILP(FlowSet& fs, FlowSet& fsUp, FlowSet& fsDn, int fanout) {
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("CBC"));
    if(!solver){ cerr<<"CBC unavailable\n"; exit(1); }

    vector<MPVariable*> var;
    buildILP(solver.get(), var, fs, fanout);

    solver->SetSolverSpecificParametersAsString("ratioGap="+std::to_string(FAT_GAP_THRESH));
    solver->Solve();

    // extract partition
    int i=1;
    for(long node: fs.nodes){
      if(var[i<<1]->solution_value()>0.5)
        fsDn.nodes.insert(node);
      else
        fsUp.nodes.insert(node);
      ++i;
    }
    fs.splitFlows(fsUp.nodes, fsUp, fsDn);
  }
};

#endif /* FATTREEP_OR_H */