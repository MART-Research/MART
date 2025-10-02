# Mapper Module Documentation

The **Mapper** module provides a suite of algorithms for mapping virtual nodes (such as tasks or logical entities) onto physical nodes in a network topology. The primary goal is to optimize communication cost, load balancing, and other criteria, depending on the chosen algorithm.

## Overview

- **Location:** `modules/Mapper/`
- **Interface:** All algorithms implement the `MappingAlgorithm` interface (see `Mapping.hpp`).
- **Excludes:** This documentation covers all mapping algorithms except for RAHTM.

## Provided Algorithms

### Greedy
A simple, fast algorithm that incrementally assigns each virtual node to the physical node that results in the lowest incremental cost, considering communication and load.

### Genetic Algorithm (GA)
Uses evolutionary strategies (selection, crossover, mutation) to search for a near-optimal mapping. Parameters include population size, generations, crossover/mutation rates, and overload penalty.

### Branch and Bound (BnB)
Systematically explores all possible mappings, pruning suboptimal branches to find the optimal solution. Suitable for small problem sizes due to combinatorial explosion.

### Ant Colony Optimization (ACO)
A metaheuristic inspired by ant foraging behavior. Multiple "ants" construct solutions using pheromone trails and heuristic information, iteratively improving the mapping.

### Particle Swarm Optimization (PSO)
A population-based stochastic optimization technique where candidate solutions ("particles") move through the solution space influenced by their own and neighbors' best positions.

### K-Nearest Neighbors (KNN)
Assigns each virtual node based on the weighted average of the physical coordinates of its k most strongly communicating (already assigned) virtual neighbors.

### Q-Learning
A reinforcement learning approach where a Q-table is used to estimate the cost of assigning each virtual node to each physical node, using an epsilon-greedy policy.

### Simulated Annealing (SA)
A probabilistic technique that explores the solution space by accepting worse solutions with a probability that decreases over time, helping to escape local minima.

### Tabu Search (TS)
An iterative local search algorithm that uses a tabu list to avoid revisiting recently explored solutions, thus encouraging exploration of new areas in the solution space.

## Common Features

- **Communication Matrix:** Most algorithms accept a communication matrix specifying the communication cost between virtual nodes.
- **Overload Penalty:** Many algorithms include a penalty for load imbalance to encourage even distribution of virtual nodes.
- **Objective Functions:** Algorithms typically optimize for total communication cost, load balancing, or a combination.

## Usage Example

```cpp
Greedy greedy_mapper;
greedy_mapper.set_comm_matrix(comm_matrix);
std::vector<int> mapping = greedy_mapper.map(vnodes, topology);
```

## Input Format Note

The input to the entire application is provided as a communication graph (comm graph), which describes the communication requirements between virtual nodes. Before mapping, a utility function is used to convert this communication graph into a communication matrix. This matrix is then used by the various mapping algorithms described above to compute optimal or near-optimal assignments.

## Extensibility

All algorithms follow the same interface, making it easy to swap or compare different mapping strategies in your experiments or applications.

---
## RAHTM Algorithm

### RAHTM (Routing Aware Hierarchial Task Mapping)
RAHTM is a specialized mapping algorithm included in the Mapper module, but it is implemented independently from the common `MappingAlgorithm` interface used by the other algorithms. RAHTM is designed for hierarchical and recursive mapping of virtual nodes onto physical topologies, often leveraging tree-based or block-based decomposition. It typically operates in multiple phases:
- **Phase 1:** Decomposes the virtual network into smaller blocks or subgraphs.
- **Phase 2:** Optimally maps each block using local search or exact methods.
- **Phase 3:** Recursively merges and re-orients blocks, using heuristics to efficiently explore the combinatorial space of possible mappings.

RAHTM is particularly effective for large-scale or highly structured mapping problems where recursive decomposition can exploit problem regularity. Its design and implementation are independent from the other mapping algorithms, allowing it to use custom data structures and optimization strategies.
