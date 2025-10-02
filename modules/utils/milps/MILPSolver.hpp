#ifndef MILP_SOLVER_HPP
#define MILP_SOLVER_HPP

#include "../utils/milps/orUtils.hpp"
#include "../utils/milps/MARTUtility.hpp"
#include <string>
#include <thread>
#include <memory>

class MILPSolver {
public:
    // Constructor initializes the solver with a specific backend (e.g., "CPLEX_MIXED_INTEGER_PROGRAMMING")
    MILPSolver(const std::string& solver_id_str) 
    : solver_(operations_research::MPSolver::CreateSolver(solver_id_str))
    {
        if (!solver_) {
            throw std::runtime_error("Could not create solver for: " + solver_id_str);
        }
    }

    // A method to configure the number of threads
    void set_num_threads(unsigned int num_threads) {
        if (solver_ && num_threads > 0) {
            // Robust Solution: Use the integer ID 1067 for the threads parameter
            solver_->SetSolverSpecificParametersAsString("1067 = " + std::to_string(num_threads));
        }
    }

    // A method to set a time limit in seconds
    void set_time_limit_seconds(double limit_seconds) {
        if (solver_) {
            solver_->SetTimeLimit(absl::Seconds(limit_seconds)); // the argument of SetTimeLimit is in milliseconds, hence * 1000
        }
    }
    // TODO
    void set_mip_gap(double gap) {

    }

    // Provide access to the underlying OR-Tools solver object for model building
    operations_research::MPSolver* get() const {
        return solver_.get();
    }

    void getSolutionLog(const operations_research::MPSolver::ResultStatus result_status, operations_research::MPObjective* const objective, std::stringstream &solverLog) {
        solverLog << "## Google OR-Tools version : " << operations_research::OrToolsVersion::VersionString() << std::endl;
        solverLog << "## Solving with " << solver_->SolverVersion() << std::endl;
        solverLog << "## Number of variables = " << solver_->NumVariables() << std::endl;
        solverLog << "## Number of constraints = " << solver_->NumConstraints() << std::endl;
        solverLog << "## Status: " << result_status << std::endl;   // Check that the problem has an optimal solution.
        if (result_status != operations_research::MPSolver::OPTIMAL) {
            solverLog << "The problem does not have an optimal solution!" << std::endl;
            if (result_status == operations_research::MPSolver::FEASIBLE)
                solverLog << "A potentially suboptimal solution was found" << std::endl;
            else
                LOG(WARNING) << "The solver could not solve the problem.";
            return;
        }
        solverLog << "## Objective value = " << objective->Value() << std::endl;
        solverLog << "## ----- Advanced usage -----" << std::endl;
        solverLog << "## Problem solved in " << solver_->wall_time() << " milliseconds" << std::endl;
        solverLog << "## Problem solved in " << solver_->iterations() << " iterations" << std::endl;
    }

private:
    std::unique_ptr<operations_research::MPSolver> solver_;
};

#endif // MILP_SOLVER_HPP
