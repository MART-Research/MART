# Application Module Documentation

## Overview

`Application.cpp` is the main entry point for the MART-New system. It orchestrates the configuration loading, topology and mapping setup, routing, and result collection for experiments and simulations in high-performance and distributed network environments. The application supports a variety of topologies, mapping algorithms, and routing strategies, and is designed to be extensible for new research and evaluation scenarios.

## Main Responsibilities
- Load configuration files for system, mapping, power, and MILP parameters.
- Instantiate and configure network topologies (Mesh, Torus, Dragonfly, etc.).
- Select and execute mapping algorithms (ACO, BnB, Greedy, GA, KNN, PSO, QLearning, SA, TS).
- Aggregate and process communication graphs.
- Initialize and run routing algorithms per virtual channel (VC) and iteration.
- Collect and log statistics (hopcount, power, max load, etc.) to CSV and log files.
- Support for MILP-based advanced algorithms (IMR, SGR, BSOR, etc.) via commented code sections.
- Modular design for easy extension and integration with new algorithms or topologies.

## Key Components
- **Configuration Loading**: Uses `ConfigLoader` to read parameters from config files for flexible experiment setup.
- **Topology Instantiation**: Dynamically creates the required network topology based on configuration.
- **Mapping Algorithms**: Supports multiple mapping techniques, each encapsulated in its own class.
- **Routing Algorithms**: Supports per-VC and per-iteration routing algorithm selection, including custom and round-robin modes.
- **Power Modeling**: Integrates a detailed power model for energy-aware evaluation.
- **MILP Section**:  Advanced MILP-based mapping and routing for research and benchmarking.
- **Result Logging**: Outputs results to CSV and log files for analysis and visualization.

## Usage
1. Prepare configuration files in `app/input/` (e.g., `mapper.config`, `power.config`, `milp.config`).
2. Place communication graph files in the appropriate input directory.
3. Build the project using the provided CMake or Makefile.
4. Run the application, optionally passing a config file as a command-line argument.
5. Results are written to `app/output/results.csv` and `app/output/log.txt`.

## Extensibility
- New topologies, mapping, or routing algorithms can be added by implementing the required interface and including them in the main application logic.
- The modular structure allows for easy integration of new research ideas and evaluation metrics.

## File Structure
- `Application.cpp`: Main application logic.
- `app/input/`: Configuration and input files.
- `app/output/`: Output logs and results.
- `modules/`: Implementation of topologies, mappers, routers, and utilities.

## Example
To run an experiment with a mesh topology and ACO mapping:
1. Edit `app/input/mapper.config` to set `mapping_technique = aco` and specify other parameters.
2. Edit `app/input/topology.config` to set `topology = mesh` and dimensions.
3. Run the application: check `how_to_run.txt`
4. Check results in `app/output/results.csv` and `app/output/log.txt`.

## See Also
- `mapper.md` for mapping module documentation.
- `router.md` for routing module documentation.
- `imr.md` for MILP-based integrated mapping and routing.
