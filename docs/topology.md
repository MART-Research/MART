# Topology Module Documentation

The **Topology** module is responsible for representing and managing the physical network structure onto which virtual nodes and communication flows are mapped. It provides the data structures and utilities needed to describe, manipulate, and query the network's nodes, links, and their interconnections.

## Overview

- **Location:**`modules/utils/networks/`
- **Utilities:** Uses supporting code from `modules/utils/links/` and `modules/utils/nodes/`

## Core Responsibilities

- Define the physical network topology (e.g., mesh, torus, custom graphs).
- Provide interfaces for querying node and link properties.
- Support the mapping and routing modules by supplying the underlying network structure.

## Key Components

### 1. Topology Representation
- The main topology classes and functions are found in `modules/utils/networks/`.
- These components allow the creation of various network topologies, such as 2D meshes, tori, or user-defined graphs.
- The topology is typically represented as a graph, with nodes (vertices) and links (edges) describing the connectivity.

### 2. Nodes Utility (`modules/utils/nodes/`)
- Provides classes and functions for representing and managing network nodes (routers, switches, etc.).
- Handles node attributes such as IDs, coordinates, and types.
- Supports operations like node creation, lookup, and property queries.

### 3. Links Utility (`modules/utils/links/`)
- Supplies classes and functions for representing network links (edges between nodes).
- Manages link attributes such as source/destination nodes and maximum load.
- Enables link creation, enumeration, and property access.

## How It Works

1. **Initialization:**
   - The topology is initialized based on configuration files (e.g., `topology.config`).
   - Nodes and links are created using the utilities from `modules/utils/nodes/` and `modules/utils/links/`.
2. **Usage:**
   - The topology object is passed to mapping and routing algorithms, which use it to determine valid placements and paths.
   - Algorithms can query node and link properties, enumerate neighbors, and compute distances or shortest paths as needed.

## Special Parameters: Faulty Links and Virtual Channels (VCs)

- **Faulty Links:**
  - The topology module supports the modeling of faulty or disabled links in the network. The percentage or list of faulty links can be specified as a parameter during topology creation (e.g., in the mesh topology constructor or configuration file). This allows simulation and analysis of network robustness and the impact of link failures on routing and mapping.
  - Algorithms and utilities can query the status of each link to determine if it is operational or faulty, ensuring that only valid paths are considered during mapping and routing.

- **Number of Virtual Channels (VCs):**
  - The number of virtual channels per physical link is another configurable parameter.
  - The topology module allows specifying the number of VCs for each link or globally for the network. This information is used by routing algorithms and can influence mapping decisions, as more VCs provide greater flexibility and bandwidth.

## Extensibility

- The modular design allows for easy addition of new topology types or custom node/link attributes.
- Utilities in `utils/nodes/` and `utils/links/` can be extended to support new features or network models.

## Example

```cpp
// Example: Creating a mesh topology and querying node neighbors
Topology * mesh_topology = mesh(4,4,n,m) // where n is number of vcs , m is percentage of faulty links
const auto& neighbors = mesh_topology.get_neighbors(node_id);
```

## Summary

The Topology module, together with its supporting utilities for nodes and links, forms the foundation for all mapping and routing operations in the system. It abstracts the physical network, enabling algorithms to work with a consistent and extensible interface for network structure and properties.
## Note
The current topology and mapping framework operates at the flow level and does not simulate buffer occupancy, VC allocation, or packet-level contention. It assumes that VCs are always available for new flows and does not model deadlock or dynamic blocking. This abstraction is suitable for high-level mapping and load estimation.