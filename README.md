# MART: Mapping and Routing Tool

MART (Mapping and Routing Tool) is a modular framework for research and experimentation in mapping and routing algorithms for high-performance and distributed network environments. It supports a wide range of network topologies, mapping algorithms, and routing strategies, and is designed for extensibility and integration with new research ideas.

---

## Table of Contents
- [Overview](#overview)
- [Modules](#modules)
  - [Application](#application)
  - [Topology](#topology)
  - [Mapper](#mapper)
  - [Router](#router)
  - [IMR (Integrated Mapping and Routing)](#imr-integrated-mapping-and-routing)
- [File Structure](#file-structure)
- [How to Run](#how-to-run)
- [Extensibility](#extensibility)
- [Examples](#references)

---

## Overview
MART provides a unified platform for:
- Defining and instantiating network topologies (Mesh, Torus, Dragonfly, Tree, etc.)
- Mapping virtual nodes (tasks) to physical nodes using various algorithms (ACO, GA, Greedy, etc.)
- Routing flows between mapped nodes with multiple strategies (DOR, Destination-Tag, Valiant, etc.)
- Modeling power, faults, and advanced MILP-based optimization
- Logging and analyzing results for research and evaluation

---

## Modules

### Application
- **Main entry point:** Orchestrates configuration loading, topology/mapping/routing setup, and result collection.
- **Configurable:** Reads from flexible config files for experiments.
- **Extensible:** Modular design for adding new algorithms or metrics.

### Topology
- **Location:** `modules/utils/networks/`
- **Purpose:** Represents the physical network structure (nodes, links, VCs, faults).
- **Features:** Supports mesh, torus, custom graphs, and models faulty links and virtual channels.

### Mapper
- **Location:** `modules/Mapper/`
- **Purpose:** Maps virtual nodes to physical nodes using algorithms like Greedy, GA, BnB, ACO, PSO, KNN, Q-Learning, SA, TS.
- **Features:** Optimizes for communication cost, load balancing, and supports various objective functions.

### Router
- **Location:** `modules/Router/`
- **Purpose:** Determines routing paths for flows using algorithms like DOR, Destination-Tag, Xor-Tag, Valiant, LBOR, etc.
- **Features:** Topology-aware, fault-aware, and supports load balancing and randomized routing.

### IMR (Integrated Mapping and Routing)
- **Location:** `modules/IMR/`
- **Purpose:** Implements OptIMaR, a MILP-based integrated mapping and routing algorithm for optimal performance and fault-tolerance.
- **Features:** Progressive recovery after failures, application-specific optimization, extensible for new strategies.

---

## File Structure
- `Application.cpp`: Main application logic
- `app/input/`: Configuration and input files
- `app/output/`: Output logs and results
- `modules/`: Implementation of topologies, mappers, routers, and utilities
- `docs/`: Detailed documentation for each module
- `examples/`: Example experiments and configurations

---

## How to Run

### Prerequisites
- C++ compiler (GCC, Clang, or MSVC)
- [CMake](https://cmake.org/) (recommended) or GNU Make
- [OR-Tools](https://developers.google.com/optimization/install/cpp/binary_windows) for MILP-based algorithms

### Building and Running

#### How To Run MART

- **Install OR-Tools as per the official documentation:**  
  https://developers.google.com/optimization/install/cpp/binary_windows
- Carefully follow the instructions to download and set up OR-Tools for C++ development.
- Ensure you have the correct version of Visual Studio installed, following the instructions in the link.
- Place the path for OR-Tools binaries in `CMAKE_PREFIX_PATH` in `CMakeLists.txt`.
- **If you are using VSCode**, include the following paths in your `c_cpp_properties.json` to avoid VSCode "Include Paths" errors:
  ```json
  "/path/to/ortools/include",
  "/path/to/ortools/include/absl"
  ```
  
In the project's main directory, you can use the following commands to build and run the project:

---

#### How to run on Windows

- **Clean Build Directory:**
  ```powershell
  Remove-Item -Recurse -Force build
  ```
- **Configure for Visual Studio 2022 (required for OR-Tools):**
  ```powershell
  cmake -G "Visual Studio 17 2022" -A x64 -B build
  ```
- **Build:**
  ```powershell
  cmake --build build --config Release
  ```
- **Run:**
  - For standard mode:
    ```powershell
    cmake --build build --target run --config Release
    ```
  - For MILP mode:
    ```powershell
    cmake --build build --target run-milp --config Release
    ```

---

#### How to run on Linux
> **Important Tip:** Ensure the config files are opened with the end-of-line sequence set to "LF" (in VSCode, see the right of the bottom bar) to avoid unexpected file parsing errors due to invisible carriage return problems.
- **Clean Build Directory:**
  ```sh
  rm -rf build
  ```
- **Configure Build Options:**
  ```sh
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  ```
  - To run using CPLEX:
  ```sh
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCPLEX_ROOT=/your/cplex/directory/path
  ```
- **Build:**
  ```sh
  cmake --build build
  ```
- **Run:**
  - For standard mode:
    ```sh
    cmake --build build --target run
    ```
  - For MILP mode:
    ```sh
    cmake --build build --target run-milp
    ```

## Extensibility
- Add new topologies, mapping, or routing algorithms by implementing the required interface and including them in the main application logic.
- Modular structure allows for easy integration of new research ideas and evaluation metrics.

---

## Examples
- Example configurations and results are provided in the `examples/` directory.
---

For more information, see the documentation in the `docs/` folder.
