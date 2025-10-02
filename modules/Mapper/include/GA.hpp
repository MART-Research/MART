#ifndef GA_MAPPING_HPP
#define GA_MAPPING_HPP

#include "../Mapping.hpp"
#include <vector>

// GA_Mapping implements a Genetic Algorithm for mapping virtual nodes to physical nodes.

class GA : public MappingAlgorithm {
public:
    // Constructor with GA parameters:
    // population_size: number of candidate solutions.
    // generations: number of generations to run.
    // crossover_rate: probability to perform crossover.
    // mutation_rate: probability to mutate a gene.
    // overload_penalty: penalty factor for load imbalance.
    // deterministic: if true, use deterministic (seeded) RNG; if false, use random device.
    // seed: seed for deterministic mode (ignored if deterministic is false).
    GA(int population_size = 50, int generations = 1000,
       double crossover_rate = 0.8, double mutation_rate = 0.1,
       double overload_penalty = 10.0, bool deterministic = false, unsigned int seed = 42u)
        : population_size_(population_size), generations_(generations),
          crossover_rate_(crossover_rate), mutation_rate_(mutation_rate),
          overload_penalty_(overload_penalty), deterministic_(deterministic), seed_(seed) {}

    // Set the communication matrix. It is assumed to be symmetric.
    void set_comm_matrix(const std::vector<std::vector<double>>& comm_matrix) {
        comm_matrix_ = comm_matrix;
    }

    // Optionally change the seed and deterministic mode at runtime.
    void set_seed(unsigned int seed) { seed_ = seed; }
    void set_deterministic(bool deterministic) { deterministic_ = deterministic; }

    // Map virtual nodes to physical nodes.
    std::vector<int> map(const std::vector<VirtualNode>& vnodes,
                         const Topology& topology) override;
private:
    int population_size_;
    int generations_;
    double crossover_rate_;
    double mutation_rate_;
    double overload_penalty_;
    bool deterministic_;
    unsigned int seed_;
    std::vector<std::vector<double>> comm_matrix_;

    // Objective function: computes total cost (communication cost + load imbalance penalty).
    double objective(const std::vector<int>& mapping, const Topology& topology);
};

#endif // GA_MAPPING_HPP
