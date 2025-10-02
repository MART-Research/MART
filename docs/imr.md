# IMR Module Documentation

The **IMR** (Integrated Mapping and Routing) module implements advanced algorithms for optimizing both mapping and routing in network-on-chip and distributed systems. Its primary algorithm, **OptIMaR**, provides an integrated, optimal approach to mapping and routing, with special support for application-specific fault-tolerance and progressive recovery after hardware failures.

## 1. Introduction

Communication is a critical factor in parallel and distributed systems, affecting correctness, performance, and energy efficiency. Many applications, such as streaming and message-passing workloads, have stable communication patterns that can be exploited for more effective orchestration. However, traditional abstraction layers often obscure this global view, making it difficult to optimize communication holistically.

Mapping (assigning threads to nodes) and routing (choosing network paths) are interdependent problems. Prior work has typically solved them separately (decoupled optimization) or used heuristic, iterative approaches. OptIMaR is the first to formulate and solve the mapping-and-routing problem in an integrated, optimal fashion using a mixed integer linear program (MILP).

## 2. Functional Description: OptIMaR Algorithm

**OptIMaR** jointly optimizes the assignment of virtual nodes to physical nodes (mapping) and the routing of flows between them. Its objectives are:
- Minimize communication cost and/or balance network load.
- Achieve optimal mapping and routing, improving performance over decoupled or heuristic approaches.
- Provide application-specific fault-tolerance by re-optimizing after link or router failures.

### Key Features:
- **Integrated MILP Formulation:** OptIMaR casts the combined mapping and routing problem as a MILP, solved using standard solvers. This enables globally optimal solutions for both mapping and routing.
- **Progressive Recovery:** After hardware failures (links/routers), OptIMaR can incrementally "heal" the system by re-optimizing mapping and routing, improving performance recovery beyond application-oblivious techniques.
- **Application-Specific Fault-Tolerance:** Unlike generic fault-tolerance, OptIMaR leverages knowledge of the application's communication pattern, only requiring connectivity between communicating pairs, not all-to-all.
- **Incremental Healing:** OptIMaR can use fast, application-oblivious recovery as a starting point, then progressively improve performance as the MILP is solved.
- **Hardware/Software Integration:** OptIMaR uses per-flow table-based routing in hardware, similar to other application-specific routing techniques.

## 3. Internal Structure

- **Algorithm Implementation:** The core logic of OptIMaR is implemented in the `modules/IMR/` directory, with separate files for MILP formulation, solution, and utility functions.
- **Integration:** The IMR module is used after the communication graph is defined, and can be invoked for both initial mapping/routing and for recovery after failures.

## 4. Usage

To use the IMR module and the OptIMaR algorithm:
1. Prepare the communication graph and physical topology.
2. Instantiate the IMR module and configure OptIMaR as needed.
3. Call the main interface to compute the optimal mapping and routing.
4. In case of failures, re-invoke OptIMaR for progressive recovery.

## 5. Extensibility

The IMR module is designed to be extensible, allowing for the addition of new integrated mapping-and-routing algorithms or fault-tolerance strategies. Its modular structure supports easy integration with other system components, such as the Mapper and Router modules.

---

