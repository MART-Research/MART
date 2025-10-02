# Router Module Documentation

The **Router** module provides algorithms and utilities for determining the paths that data flows take through the physical network topology. Its primary goal is to ensure efficient and valid routing of flows between mapped nodes, supporting a variety of topologies and routing strategies.

## Overview

- **Location:** `modules/Router/`
- **Interface:** All routing algorithms are accessible via the main router interface (see `Router.hpp`).
- **Integration:** The Router module is used after mapping virtual nodes to physical nodes and works closely with the Topology module.

## Provided Algorithms

The Router module implements several routing algorithms, each tailored for different topologies and requirements. Most algorithms follow the main Routing interface, but some (BSOR, Transcom, SGR) are implemented independently due to their unique logic.

### Dimension-Order Routing (DOR)
A deterministic algorithm (also known as XY-routing in 2D meshes) that routes packets by traversing one dimension at a time. Commonly used in mesh and torus topologies for its simplicity and deadlock avoidance properties.

### Destination-Tag Routing (DTR)
Destination-Tag Routing uses tags derived from the destination node to guide routing decisions. The tag format depends on the topology:
- **Mesh/Torus/Ring:** Tags encode destination coordinates (row, column, depth). Routing proceeds dimension by dimension, using the tag to determine the next hop.
- **Hypercube:** Tags are bitwise (XOR of source and destination), indicating which dimensions differ.
- **Dragonfly:** Tags encode hierarchical information (group, router).
- **Tree/Butterfly:** Tags may encode the full path or use BFS/DFS to precompute the route.

The algorithm is topology-aware and updates link loads as it routes. It is fault-aware and will return an empty path if a link is down.

### Xor-Tag Routing (Xor)
Xor-Tag Routing is designed for topologies like hypercubes and butterfly networks. It uses the XOR of source and destination addresses to determine which dimensions need to be traversed:
- At each step, the algorithm flips the lowest-order differing bit between the current node and the destination, moving closer in Hamming distance.
- For butterfly networks, the algorithm uses the XOR tag to determine the switch at each stage.

This approach ensures minimal path routing in hypercube-like topologies and is robust to faults (returns an empty path if a link is down).

### Valiant
Valiant Routing is a two-phase randomized routing algorithm for load balancing:
- The route is split into two segments: from source to a randomly chosen intermediate node, then from the intermediate node to the destination.
- For mesh/torus/ring, it uses dimension-order routing for each segment.
- For hypercube and dragonfly, it uses topology-specific routing for each segment.
- The algorithm is fault-aware and will return an empty path if any segment is blocked.

This method helps avoid congestion and balances network load, at the cost of potentially longer paths.

### Load Balanced Oblivious Routing (LBOR)
LBOR is an oblivious routing algorithm that introduces randomization to balance load:
- For mesh/torus/ring, it selects a random intermediate node within a specific quadrant between source and destination, then routes via dimension-order routing.
- For hypercube, it selects a random intermediate node and routes via bitwise moves.
- The algorithm checks for direct links and uses them if available.
- Fault-aware: returns an empty path if a link is down.

LBOR is designed to reduce hotspots and improve throughput under uniform and adversarial traffic.

### Minimal Oblivious Routing (MinimalOR)
MinimalOR enumerates all minimal (shortest) paths between source and destination:
- For mesh/torus/ring, it generates all minimal permutations of moves in each dimension, considering wrap-around for torus.
- For hypercube, it uses bitwise moves.
- For dragonfly and butterfly, it uses BFS or backtracking to find minimal paths.
- One minimal path is selected at random for routing.
- Fault-aware: only valid, non-faulty paths are considered.

This approach supports multipath routing and load balancing by providing all minimal options, however the choice is random.

### BSOR
BSOR (Bandwidth Sensitive Oblivious Routing) is a MILP-based algorithm:
- It constructs a communication dependency graph (CDG) and transforms it into an acyclic CDG (ACDG) using turn models (e.g., North-Last for 2D mesh).
- The algorithm generates a flow network on top of the ACDG and solves a MILP to find optimal (balanced) routing assignments for all flows.
- It is designed for topologies where deadlock avoidance and load balancing are critical.
- The implementation is independent of the main router interface due to its unique optimization-based logic.

### Transcom
Transcom is an advanced compile-time routing optimization that leverages application-specific communication patterns to improve network performance and load balance. It applies two key transformations:
- **Fission:** Splits a bottleneck flow into multiple flows, improving load balance by distributing traffic across multiple paths.
- **Fusion:** Merges multiple flows used for multicast communication into a multicast distribution tree, reducing redundant channel usage.

Transcom uses free routing (without deadlock-avoidance restrictions) to maximize the benefits of these transformations. If free routing introduces cycles in the channel dependence graph (CDG), Transcom attempts to break them using virtual channels (VCs), formulating the deadlock-free VC assignment as an integer linear program (ILP). If deadlocks cannot be avoided with the available VCs, Transcom can fall back to a more restricted routing mode, still outperforming traditional BSOR.

Transcom is especially effective for network-bound applications, optimizing the maximum channel load (MCL) to improve throughput. It is implemented independently from the main router interface due to its unique compile-time optimization logic.

### SGR
SGR (Scalable Global Routing) is a two-phase compile-time routing algorithm designed for scalability and optimal load balancing in large networks. Its key innovations are:

- **Amorphous Routing:** In the first phase, SGR computes the optimal channel loads on each link without explicitly identifying the end-to-end paths that contribute to these loads. This allows for scalable and globally optimal load distribution, as the routing problem is decoupled from the enumeration of individual paths.

- **Flow Peeling:** In the second phase, SGR reconstructs end-to-end routing paths and their corresponding weights from the amorphous routing solution. This is achieved using an iterative greedy algorithm that "peels off" flows from the channel load assignments, producing concrete paths suitable for hardware implementation (e.g., routing tables or source-routing headers).

This decoupling enables SGR to exploit multipath routing and achieve better load balance than traditional approaches. SGR is especially effective for large-scale, network-bound applications where scalable and optimal routing is critical. It is implemented independently from the main router interface due to its unique two-phase logic.

## Common Features

- **Fault Awareness:** Routing algorithms check the status of links and nodes to avoid faulty components, ensuring valid paths even in the presence of failures.
- **Flow-Level Routing:** The module operates at the flow level, assuming that flows are routed sequentially and that resources (VCs, links) are always available when needed. It does not simulate buffer contention, packet-level dynamics, or deadlock.
## Usage Example

```cpp
Router router(topology);
auto path = router.route(src_id, dst_id, hopcount, vc, flow);
```

## Input Format Note

The Router module receives the physical topology and the set of flows to be routed after the mapping step. Users can select the routing algorithm and parameters. The module returns the computed paths, which can be used for further analysis, simulation, or output.

## Extensibility

The modular design allows for new routing algorithms or strategies to be added with minimal changes. The Router module is designed to be easily extended to support additional topologies or routing policies as needed.

---

## Summary

The Router module provides the essential logic for determining how data moves through the network, supporting a range of topologies and routing strategies. It is designed for high-level, flow-based analysis and integrates tightly with the mapping and topology components of the system.
