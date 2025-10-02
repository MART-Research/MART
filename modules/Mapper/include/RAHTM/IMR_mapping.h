#ifndef IMR_OR_TOOLS_H
#define IMR_OR_TOOLS_H

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "flowset.h"
#include "point.h"
#include "nodemappings.h"

#include "ortools/base/logging.h"
#include "ortools/linear_solver/linear_solver.h"

using namespace std;
using namespace operations_research;

#define GAP_THRESH 0.001

map<long, map<long, map<long, long long>>> f_fl_edge_ind;
map<long, map<long, long long>>            g_rank_node;
long long last_f_ind; // last flow index

double find_min_Z(FlowSet& fs, point& dims)
{
    map<long, double> src_out, dest_in;

    for (auto& [u, dests] : fs.flows)
        for (auto& [v, wt] : dests) {
            src_out[u]  += wt;
            dest_in[v]  += wt;
        }

    double max_sum = 0.0;
    for (auto& [_, s] : src_out)  max_sum = max(max_sum, s);
    for (auto& [_, s] : dest_in)  max_sum = max(max_sum, s);

    return max_sum / static_cast<double>(dims.size());
}

static void build_ilp(MPSolver* solver, vector<MPVariable*>& x, FlowSet& fs, point& dims)
{
    int   conc_factor      = 1; 
    long  f_priority_wt    = fs.n_flows * 10; 
    double min_Z           = find_min_Z(fs, dims);

    /* Z (index 0) */
    x.emplace_back(solver->MakeNumVar(min_Z, solver->infinity(), "Z"));

    last_f_ind = 0;
    auto* obj  = solver->MutableObjective();
    // O2
    point p;
    do {
        map<pair<int,int>, point> neighbors;
        p.get_neighbors(neighbors, dims);
        long p_val = p.get_value(dims);

        for (auto& [edge, nb_pt] : neighbors) {
            long neighbor = nb_pt.get_value(dims);

            MPConstraint* expr = solver->MakeRowConstraint(0.0, solver->infinity());
            expr->SetCoefficient(x[0], 1.0);   // +Z
            obj->SetCoefficient(x[0], 1.0);    // objective accumulates +Z

            long flow = 0;
            for (auto& [u, dests] : fs.flows)
                for (auto& [v, wt] : dests) {
                    stringstream f_str; f_str << "f_" << flow << "N_"; p.write_point(f_str); f_str << "N_"; nb_pt.write_point(f_str);

                    ++last_f_ind;
                    x.emplace_back(solver->MakeNumVar(0.0, wt, f_str.str()));

                    f_fl_edge_ind[flow][p_val][neighbor] = last_f_ind;

                    expr->SetCoefficient(x[last_f_ind], -1.0);
                    obj ->SetCoefficient(x[last_f_ind], f_priority_wt);
                    ++flow;
                }
        }
    } while (p.inc(dims));

    obj->SetMinimization();

    // C1-2
    p.set(0);
    do {
        MPConstraint* expr = solver->MakeRowConstraint(-solver->infinity(), conc_factor);

        for (long r : fs.nodes) {
            stringstream g_str; g_str << "g_" << r << '_' << p.get_value(dims);
            ++last_f_ind;
            x.emplace_back(solver->MakeBoolVar(g_str.str()));

            g_rank_node[r][p.get_value(dims)] = last_f_ind;
            expr->SetCoefficient(x[last_f_ind], 1.0);
        }
    } while (p.inc(dims));

    // C2
    p.set(0);
    do {
        map<pair<int,int>, point> neighbors;
        p.get_neighbors(neighbors, dims);
        long p_val = p.get_value(dims);

        long flow = 0;
        for (auto& [u, dests] : fs.flows) {
            for (auto& [v, wt] : dests) {
                MPConstraint* expr = solver->MakeRowConstraint(0.0, 0.0);

                for (auto& [edge, nb_pt] : neighbors) {
                    long neighbor = nb_pt.get_value(dims);

                    long long f_out = f_fl_edge_ind[flow][p_val][neighbor];
                    long long f_in  = f_fl_edge_ind[flow][neighbor][p_val];

                    expr->SetCoefficient(x[f_out],  1.0);
                    expr->SetCoefficient(x[f_in],  -1.0);
                }
                expr->SetCoefficient(x[g_rank_node[u][p_val]], -wt);
                expr->SetCoefficient(x[g_rank_node[v][p_val]],  wt);
                ++flow;
            }
        }
    } while (p.inc(dims));

    // C1-1
    for (long r : fs.nodes) {
        MPConstraint* expr = solver->MakeRowConstraint(1.0, 1.0);
        point pp;
        do {
            expr->SetCoefficient(x[g_rank_node[r][pp.get_value(dims)]], 1.0);
        } while (pp.inc(dims));
    }
}

inline void imr_map(FlowSet& fs, point& dims, NodeMappings& nm)
{
    f_fl_edge_ind.clear(); g_rank_node.clear();

    unique_ptr<MPSolver> solver(MPSolver::CreateSolver("CBC"));
    CHECK(solver) << "Could not create CBC solver";

    vector<MPVariable*> vars;
    build_ilp(solver.get(), vars, fs, dims);

    // solver->SetSolverSpecificParametersAsString("ratioGap=0.001");
    // solver->SetTimeLimit(180000);

    solver->Solve();

    double MCL = vars[0]->solution_value();
    cout << "----------------------- MCL = " << MCL << " -----------------------\n";

    /* Extract mapping */
    for (auto& [rank, nodeMap] : g_rank_node)
        for (auto& [proc, idx] : nodeMap)
            if (vars[idx]->solution_value() > 0.5) {
                nm.mapping[rank] = point(proc, dims);
                break;
            }
}

#endif  /* IMR_OR_TOOLS_H */
