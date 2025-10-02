#include "../../modules/utils/networks/topologies/Mesh.hpp"
#include "../../modules/utils/networks/topologies/Ring.hpp"
#include "../../modules/utils/networks/topologies/Hypercube.hpp"
#include "../../modules/utils/networks/topologies/Torus.hpp"
#include "../../modules/utils/networks/topologies/Tree.hpp"
#include "../../modules/utils/networks/topologies/Dragonfly.hpp"
#include "../../modules/utils/networks/topologies/Butterfly.hpp" 
#include "../../modules/Router/include/DimensionOrder.hpp"
// #include "../../modules/Router/include/VariableDimensionOrder.hpp"
#include "../../modules/Router/include/DestinationTag.hpp"
#include "../../modules/Router/include/Valiant.hpp"
#include "../../modules/Router/include/MinimalOblivious.hpp"
#include "../../modules/Router/include/XorTag.hpp"
#include "../../modules/Router/include/LoadBalancedOblivious.hpp"
#include "../../modules/Mapper/include/ACO.hpp"
#include "../../modules/Mapper/include/ACO_deterministic.hpp"
#include "../../modules/Mapper/include/BnB.hpp"
#include "../../modules/Mapper/include/GA.hpp"
#include "../../modules/Mapper/include/Greedy.hpp"
#include "../../modules/Mapper/include/KNN.hpp"
#include "../../modules/Mapper/include/PSO.hpp"
#include "../../modules/Mapper/include/QLearning.hpp"
#include "../../modules/Mapper/include/SA.hpp"
#include "../../modules/Mapper/include/TS.hpp"
#include "../include/config_loader.hpp"
#include "../../modules/utils/nodes/map_nodes.hpp"
#include "../../modules/Router/include/BSOR.hpp"
#include "../../modules/Router/include/TransCom.hpp"
#include "../../modules/IMR/include/IMR.hpp"
#include "../../modules/Router/include/SGR.hpp"
#include "../../modules/utils/milps/topology_interface.hpp"
#include "../../modules/utils/milps/MARTUtility.hpp"
#include "../../modules/utils/milps/MILPSolver.hpp"
#include "../../modules/utils/power/power_model.hpp"
#include "../../modules/utils/links/Link.hpp"
#include "../include/helper_functions.hpp"
#include <iostream>
#include <string>
#include <unordered_map>
#include <sstream>
#include <vector>
#include <tuple>
#include <map>
#include <utility>
#include <algorithm>
#include <fstream>
#include <filesystem>

// RAHTM utilizes POSIX threads, which are only availble through a unix-like OS
#if defined(__linux__) || defined(__APPLE__)
#include "../../modules/Mapper/include/RAHTM/RAHTM.hpp"
#endif

#define NORMALIZATION_VALUE 1e9
// Normalization factor for power calculations
// This value is used to normalize power values to a manageable scale
double normalization_factor = 1;
 
int main(int argc, char *argv[])
{
    std::string config_file;
    bool milp_mode = false;
    if (argc == 2)
    {
        config_file = argv[1];
    }
    else if (argc == 3)
    {
        config_file = argv[1];
        std::string milp_mode_flag = argv[2];
        if (milp_mode_flag == "--milp")
        {
            // cout << "MILP mode enabled." << endl;
            milp_mode = true;
        }
        else
        {
            std::cerr << "Error: Invalid argument '" << milp_mode_flag << "'. Use '--milp' to enable MILP mode." << std::endl;
            return 1;
        }
    }
    else
    {
        // error message for incorrect usage
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return 1;
    }

    // Load the configuration file
    ConfigLoader main_config;
    try
    {
        main_config.load_config(config_file);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: Failed to load configuration file: " << e.what() << std::endl;
        return 1;
    }
    // Load parameters from the configuration file
    std::string power_model_mode = main_config.get_param("power_model_mode", "none");
    std::string power_model_file = "";
    if (power_model_mode == "detailed")
    {
        power_model_file = "app/input/power.config";
    }
    else if (power_model_mode != "none")
    {
        std::cerr << "Error: Unsupported power model mode: " << power_model_mode << std::endl;
        return 1;
    }
    std::string topology_name = main_config.get_param("topology", "mesh");
    int num_vcs = std::stoi(main_config.get_param("num_vcs", "4"));
    int num_iterations = std::stoi(main_config.get_param("num_iterations", "1"));
    float faulty_links = std::stof(main_config.get_param("percentage_of_faulty_links", "0.0"));
    int num_groups = 0;
    int routers_per_group = 0;
    Topology *topology = nullptr;
    // Initialize the topology based on the configuration and the topology name where each topology requires different parameters.
    if (topology_name == "butterfly")
    {
        int k;
        k = std::stoi(main_config.get_param("k", "1"));
        topology = new Butterfly(k, num_vcs, faulty_links);
    }
    else if (topology_name == "tree")
    {
        int num_levels;
        int branching_factor;
        num_levels = std::stoi(main_config.get_param("num_levels", "1"));
        branching_factor = std::stoi(main_config.get_param("branching_factor", "1"));
        topology = new Tree(num_levels, branching_factor, num_vcs, faulty_links);
    }
    else if (topology_name == "dragonfly")
    {
        int nodes_per_router;
        int global_links;
        num_groups = std::stoi(main_config.get_param("num_groups", "1"));
        routers_per_group = std::stoi(main_config.get_param("routers_per_group", "1"));
        nodes_per_router = std::stoi(main_config.get_param("nodes_per_router", "1"));
        global_links = std::stoi(main_config.get_param("global_links", "1"));
        topology = new Dragonfly(num_groups, routers_per_group, nodes_per_router, global_links, num_vcs, faulty_links);
    }
    else if (topology_name == "mesh")
    {
        int rows = std::stoi(main_config.get_param("topology_rows", "3"));
        int cols = std::stoi(main_config.get_param("topology_columns", "3"));
        topology = new Mesh(rows, cols, num_vcs, faulty_links);
    }
    else if (topology_name == "torus")
    {
        int rows = std::stoi(main_config.get_param("topology_rows", "3"));
        int cols = std::stoi(main_config.get_param("topology_columns", "3"));
        int depths = std::stoi(main_config.get_param("topology_depth", "1"));
        if (depths == 1)
            topology_name = "torus_2d";
        else if (depths > 1)
        {
            topology_name = "torus_3d";
        }
        else
        {
            std::cerr << "Error: Invalid depth for Torus topology: " << depths << std::endl;
            return 1;
        }
        topology = new Torus(rows, cols, depths, num_vcs, faulty_links);
    }
    else if (topology_name == "hypercube")
    {
        int dimensions = std::stoi(main_config.get_param("dimensions", "1"));
        topology = new Hypercube(dimensions, num_vcs, faulty_links);
    }
    else if (topology_name == "ring")
    {
        int num_nodes = std::stoi(main_config.get_param("num_nodes", "1"));
        topology = new Ring(num_nodes, num_vcs, faulty_links);
    }
    else
    {
        std::cerr << "Error: Unsupported topology type: " << topology_name << std::endl;
        return 1;
    }
    std::string vc_operation_mode = main_config.get_param("vc_operation_mode", "round_robin");
    // Initialize routing algorithms for each VC (if custom mode is enabled)///
    std::vector<std::vector<std::string>> vc_routing_algorithm(num_iterations, std::vector<std::string>(num_vcs));
    if (vc_operation_mode == "custom")
    {
        for (int iter = 0; iter < num_iterations; ++iter)
        {
            // Get the routing algorithms for this iteration
            std::string vc_routing_algorithms_str = main_config.get_param("vc_routing_algorithms_" + std::to_string(iter + 1), "");
            if (vc_routing_algorithms_str.empty())
            {
                std::cerr << "Error: No VC routing algorithms specified for iteration " << iter + 1 << std::endl;
                return 1;
            }
            std::stringstream ss(vc_routing_algorithms_str);
            std::string algorithm;
            int vc_index = 0;

            while (std::getline(ss, algorithm, ','))
            {
                // Trim whitespace from the algorithm string
                algorithm.erase(algorithm.find_last_not_of(" \n\r\t") + 1);
                algorithm.erase(0, algorithm.find_first_not_of(" \n\r\t"));

                if (vc_index < num_vcs)
                {
                    vc_routing_algorithm[iter][vc_index] = algorithm;
                    ++vc_index;
                }
                else
                {
                    std::cerr << "Error: Too many VC routing algorithms specified for iteration " << iter + 1 << std::endl;
                    return 1;
                }
            }
            // Ensure the number of algorithms matches the number of VCs
            if (vc_index != num_vcs)
            {
                std::cerr << "Error: Number of VC routing algorithms does not match num_vcs for iteration " << iter + 1 << std::endl;
                return 1;
            }
        }
    }
    else if (vc_operation_mode == "round_robin")
    {
        for (int iter = 0; iter < num_iterations; ++iter)
        {
            std::string routing_algorithm = main_config.get_param("routing_algorithm_" + std::to_string(iter + 1), "");
            if (routing_algorithm.empty())
            {
                std::cerr << "Error: No routing algorithm specified for iteration " << iter + 1 << std::endl;
                return 1;
            }
            vc_routing_algorithm[iter].assign(num_vcs, routing_algorithm); // Fill with the same algorithm for all VCs in this iteration
        }
    }
    else
    {
        std::cerr << "Error: Unsupported VC operation mode: " << vc_operation_mode << std::endl;
        return 1;
    }

    // ---------- START OF MAPPING ----------
    // Mapping parameters
    ConfigLoader mapping_config;
    mapping_config.load_config("app/input/mapper.config");
    std::string virtual_nodes_str = mapping_config.get_param("virtual_nodes", "0");
    int virtual_nodes = std::stoi(virtual_nodes_str);
    std::string mapping_technique = mapping_config.get_param("mapping_technique", "none");
    std::string comm_graph_file = mapping_config.get_param("communication_graph", "cg.64");
    comm_graph_file = "app/input/" + comm_graph_file;
    std::vector<std::vector<double>> comm_matrix;
    std::ifstream comm_graph_file_stream(comm_graph_file);
    if (!comm_graph_file_stream.is_open())
    {
        std::cerr << "Error: Unable to open communication matrix file: " << comm_graph_file << std::endl;
        return 1;
    }
    // Read the communication graph file and convert it to a comm_matrix
    std::multimap<int, std::pair<int, int>> comm_graph;
    if (virtual_nodes > 0)
    {
        // Read the communication graph from the file
        comm_graph = readFlows_singledest(comm_graph_file);
        // Convert the communication graph to a comm_matrix
        comm_matrix = comm_graph_to_matrix(comm_graph, virtual_nodes);
    }
    else
    {
        std::cerr << "Error: Number of virtual nodes must be greater than 0." << std::endl;
        return 1;
    }
    // Create virtual nodes.
    std::vector<VirtualNode> vnodes;
    std::vector<int> mapping;
    for (int i = 0; i < virtual_nodes; ++i)
    {
        vnodes.push_back({i, "VNode_" + std::to_string(i)});
    }
    if (mapping_technique == "aco")
    {
        ACO_Deterministic aco;
        aco.set_comm_matrix(comm_matrix);
        mapping = aco.map(vnodes, *topology);
    }
    else if (mapping_technique == "bnb")
    {
        BnB bnb;
        bnb.set_comm_matrix(comm_matrix);
        mapping = bnb.map(vnodes, *topology);
    }
    else if (mapping_technique == "greedy")
    {
        Greedy greedy;
        greedy.set_comm_matrix(comm_matrix);
        mapping = greedy.map(vnodes, *topology);
    }
    else if (mapping_technique == "ga")
    {
        GA ga;
        ga.set_comm_matrix(comm_matrix);
        mapping = ga.map(vnodes, *topology);
    }
    else if (mapping_technique == "knn")
    {
        KNN knn;
        knn.set_comm_matrix(comm_matrix);
        mapping = knn.map(vnodes, *topology);
    }
    else if (mapping_technique == "pso")
    {
        PSO pso;
        pso.set_comm_matrix(comm_matrix);
        mapping = pso.map(vnodes, *topology);
    }
    else if (mapping_technique == "q_learning")
    {
        QLearning q_learning;
        q_learning.set_comm_matrix(comm_matrix);
        mapping = q_learning.map(vnodes, *topology);
    }
    else if (mapping_technique == "sa")
    {
        SA sa;
        sa.set_comm_matrix(comm_matrix);
        mapping = sa.map(vnodes, *topology);
    }
    else if (mapping_technique == "ts")
    {
        TS ts;
        ts.set_comm_matrix(comm_matrix);
        mapping = ts.map(vnodes, *topology);
    }
    else
    {
        std::cerr << "Error: Unsupported mapping technique: " << mapping_technique << std::endl;
        return 1;
    }
    // ---------- END OF MAPPING ----------

    // Results file setup
    // Open a CSV file to save results in append mode so that results from multiple runs can be aggregated
    std::ofstream csv_file("app/output/results.csv", std::ios::app);
    if (!csv_file.is_open())
    {
        std::cerr << "Error: Unable to open results.csv for writing." << std::endl;
        return 1;
    }

    // Write the CSV header only if the file is empty
    std::ifstream check_file("app/output/results.csv");
    bool is_empty = check_file.peek() == std::ifstream::traits_type::eof();
    check_file.close();

    if (is_empty)
    {
        csv_file << "Topology,Algorithm,Total Hopcount,Average Hopcount,Average Power Per Route,MCL,Total Power\n";
    }

    // Power model initialization
    // Load detailed power parameters from the configuration file
    ConfigLoader power_config;
    power_config.load_config(power_model_file);

    // General Power Parameters
    double voltage = std::stod(power_config.get_param("voltage", "1.0"));     // Volts
    double frequency = std::stod(power_config.get_param("frequency", "1e9")); // Hz

    // Node Power Parameters
    double buffer_capacitance = std::stod(power_config.get_param("node_buffer_capacitance", "1e-12"));     // Farads
    double crossbar_capacitance = std::stod(power_config.get_param("node_crossbar_capacitance", "2e-12")); // Farads
    double node_leakage_current = std::stod(power_config.get_param("node_leakage_current", "2e-6"));       // Amperes

    // Link Power Parameters
    double link_capacitance = std::stod(power_config.get_param("link_capacitance", "1e-12"));        // Farads
    double link_leakage_current = std::stod(power_config.get_param("link_leakage_current", "1e-6")); // Amperes

    // Activity Factors
    double buffer_read_write_factor = std::stod(power_config.get_param("buffer_read_write_factor", "0.6"));
    double crossbar_traversal_factor = std::stod(power_config.get_param("crossbar_traversal_factor", "0.8"));
    double link_activity_factor = std::stod(power_config.get_param("link_activity_factor", "0.5"));

    if (milp_mode)
    {
        cout << "Running in MILP mode." << endl;
        // ---------- START OF MILP SECTION ----------

        ConfigLoader milp_config;
        milp_config.load_config("app/input/milp.config");
        std::string milp_solver = milp_config.get_param("milp_solver", "CBC");
        std::string algorithm_id = milp_config.get_param("algorithm_id", "sgr");
        algorithm_id.erase(std::remove_if(algorithm_id.begin(), algorithm_id.end(), [](unsigned char ch)
                                          { return std::isspace(ch); }),
                           algorithm_id.end());
        bool custom_topology = std::stoi(milp_config.get_param("custom_topology", "0"));
        std::string topology_file = milp_config.get_param("topology_file", "topology.txt");
        int concentration_factor = std::stoi(milp_config.get_param("concentration_factor", "1"));
        bool topology_is_mapped = std::stoi(milp_config.get_param("topology_is_mapped", "0"));


        // initialize MILPSolver instance
        MILPSolver my_milp_solver(milp_solver);
        my_milp_solver.set_num_threads(8);
        my_milp_solver.set_time_limit_seconds(3600); // 1 hour limit

        multiSplitFlowData data;
        double MCL = 0.0; // Initialize to avoid uninitialized variable warning
        int total_hopcount;
        size_t comm_graph_size;
        double total_power_per_routes = 0;
        if (algorithm_id == "bsor")
        {
            cout<<"Running BSOR algorithm." << endl;
            int bsor_objective = std::stoi(milp_config.get_param("bsor_objective_num", "3"));

            std::multimap<int, std::pair<int, double>> communication_graph = readFlows_singledest_double(comm_graph_file);

            if (topology_is_mapped == false)
                communication_graph = retrive_physical_mapping(communication_graph, mapping);

            comm_graph_size = communication_graph.size();

            // printCommunicationGraph(communication_graph);

            std::unordered_map<std::pair<int, int>, int, pair_hash> flowNetwork;
            if (custom_topology)
            {
                flowNetwork = readFlowNetwork_Capacity(topology_file);
            }
            else
            {
                preprocess_topology(*topology, flowNetwork);
            }
            // printFlowNetwork_Capacity(flowNetwork);
            BSOR bsor(my_milp_solver, topology->get_num_nodes(), flowNetwork, communication_graph, bsor_objective, false, topology->get_num_rows(), topology->get_num_cols());
            bsor.run_bsor();
            cout<<"BSOR algorithm completed." << endl;
            CDGFlowData data = parseCDGMILPResults("app/output/milps/bsor_milp_output.txt");
            cout << "Parsed CDGFlowData from BSOR MILP results." << endl;
            total_hopcount = data.total_hops;
            double total_throughput = 0.0;
            double min_fairness = 0.0;
            if (bsor_objective == 1)
            {
                total_throughput = data.objectives[1];
            }
            else if (bsor_objective == 2)
            {
                min_fairness = data.objectives[2];
            }
            else
            {
                MCL = data.objectives[3];
            }
            // currently neither total_throughput nor min_fairness are written to the CSV file, but they can be added if needed. They are to be found in the log file anyway.

            calculate_power_CDG_flow(data, topology, total_power_per_routes, link_capacitance, link_leakage_current,
                                     voltage, frequency, link_activity_factor,
                                     buffer_read_write_factor, crossbar_traversal_factor, buffer_capacitance, crossbar_capacitance, node_leakage_current);
            data.total_power = calculate_total_power(topology);
            csv_file << topology_name << ","
                     << algorithm_id << ","
                     << total_hopcount << ","
                     << static_cast<double>(total_hopcount) / (comm_graph_size) << ","
                     << static_cast<double>(total_power_per_routes) / (comm_graph_size) << ","
                     << MCL << ","
                     << data.total_power << "\n";
            data.mapping = mapping;
            writeResultsToFile(data, "app/output/bsor_results.txt");
        }

        else if (algorithm_id == "txcomm")
        {
            std::multimap<int, std::pair<int, double>> communication_graph = readFlows_singledest_double(comm_graph_file);

            if (topology_is_mapped == false)
                communication_graph = retrive_physical_mapping(communication_graph, mapping);

            comm_graph_size = communication_graph.size();

            std::unordered_map<int, std::vector<int>> flowNetwork;
            if (custom_topology)
            {
                flowNetwork = readFlowNetwork(topology_file);
            }
            else
            {
                preprocess_topology(*topology, flowNetwork);
            }

            TRANSCOM transcom(my_milp_solver, flowNetwork, communication_graph);
            transcom.run_transcom_fission_only();

            data = parseMILPResults_multiSplit("app/output/milps/txcomm_milp_output.txt");
            MCL = data.objectives[1];
            total_hopcount = data.objectives[2];

            calculate_power_multi_split_flow(data, topology, total_power_per_routes, link_capacitance, link_leakage_current,
                                             voltage, frequency, link_activity_factor,
                                             buffer_read_write_factor, crossbar_traversal_factor, buffer_capacitance, crossbar_capacitance, node_leakage_current);
            data.total_power = calculate_total_power(topology);
            // Write results to the CSV file
            csv_file << topology_name << ","
                     << algorithm_id << ","
                     << total_hopcount << ","
                     << static_cast<double>(total_hopcount) / (comm_graph_size) << ","
                     << static_cast<double>(total_power_per_routes) / (comm_graph_size) << ","
                     << MCL << ","
                     << data.total_power << "\n";
            data.mapping = mapping;
            writeResultsToFile(data, "app/output/txcomm_results.txt");
        }
        else if (algorithm_id == "imr")
        {
            std::cout<<"DEBUG1\n";
            std::multimap<int, std::pair<int, double>> communication_graph = readFlows_singledest_double(comm_graph_file);

            comm_graph_size = communication_graph.size();

            std::unordered_map<int, std::vector<int>> flowNetwork;
            if (custom_topology)
            {
                flowNetwork = readFlowNetwork(topology_file);
            }
            else
            {
                preprocess_topology(*topology, flowNetwork);
            }
            std::cout<<"DEBUG2\n";

            // printFlowNetwork(flowNetwork);

            IMR imr(my_milp_solver, flowNetwork, communication_graph, topology->get_num_nodes(), concentration_factor);
            std::cout<<"DEBUG3\n";
            imr.run_imr_fission_only();
            std::cout<<"DEBUG4\n";

            data = parseMILPResults_multiSplit("app/output/milps/imr_milp_output.txt");
            MCL = data.objectives[1];
            total_hopcount = data.objectives[2];

            calculate_power_multi_split_flow(data, topology, total_power_per_routes, link_capacitance, link_leakage_current,
                                             voltage, frequency, link_activity_factor,
                                             buffer_read_write_factor, crossbar_traversal_factor, buffer_capacitance, crossbar_capacitance, node_leakage_current);
            data.total_power = calculate_total_power(topology);
            // Write results to the CSV file
            csv_file << topology_name << ","
                     << algorithm_id << ","
                     << total_hopcount << ","
                     << static_cast<double>(total_hopcount) / (comm_graph_size) << ","
                     << static_cast<double>(total_power_per_routes) / (comm_graph_size) << ","
                     << MCL << ","
                     << data.total_power << "\n";
            writeResultsToFile(data, "app/output/imr_results.txt");
        }
        else if (algorithm_id == "sgr")
        {
            int sgr_objective_num = std::stoi(milp_config.get_param("sgr_objective_num", "2"));

            std::multimap<int, std::pair<int, double>> communication_graph = readFlows_singledest_double(comm_graph_file);

            if (topology_is_mapped == false)
                communication_graph = retrive_physical_mapping(communication_graph, mapping);

            comm_graph_size = communication_graph.size();

            std::unordered_map<int, std::vector<int>> flowNetwork;
            if (custom_topology)
            {
                flowNetwork = readFlowNetwork(topology_file);
            }
            else
            {
                preprocess_topology(*topology, flowNetwork);
            }

            SGR sgr(my_milp_solver, topology->get_num_nodes(), flowNetwork, communication_graph, sgr_objective_num);
            PeelPaths peelpaths = sgr.run_sgr();

            noSplitFlowData data;
            data = parseMILPResultsNoSplits("app/output/milps/sgr_milp_output.txt");
            for (size_t t = 0; t < peelpaths.paths.size(); ++t)
            {
                data.routes[t] = peelpaths.paths[t];
                data.flow_weights[t] = peelpaths.pathWeights[t];
                for (size_t k = 0; k < peelpaths.paths[t].size() - 1; ++k)
                {
                    std::pair<int, int> edge = {peelpaths.paths[t][k], peelpaths.paths[t][k + 1]};
                    data.edge_loads[edge] += peelpaths.pathWeights[t];
                }
                data.total_hops += peelpaths.paths[t].size() - 1;
            }
            total_hopcount = data.total_hops;
            data.objectives[1] = total_hopcount;
            MCL = data.objectives[2];

            calculate_power_no_splits(data, topology, total_power_per_routes, link_capacitance, link_leakage_current,
                                      voltage, frequency, link_activity_factor,
                                      buffer_read_write_factor, crossbar_traversal_factor, buffer_capacitance, crossbar_capacitance, node_leakage_current);
            data.total_power = calculate_total_power(topology);
            // Write results to the CSV file
            csv_file << topology_name << ","
                     << algorithm_id << ","
                     << data.total_hops << ","
                     << static_cast<double>(data.total_hops) / (comm_graph_size) << ","
                     << static_cast<double>(total_power_per_routes) / (comm_graph_size) << ","
                     << MCL << ","
                     << data.total_power << "\n";
            data.mapping = mapping;
            std::cout << data.mapping.size() << std::endl;
            writeResultsToFileNoSplits(data, "app/output/sgr_results.txt");
        }
        else if (algorithm_id == "rahtm")
        {
// RAHTM can only run on unix-like OS, so we check for that
#if defined(__linux__) || defined(__APPLE__)
            int sz = std::stoi(milp_config.get_param("process_count", "256"));
            bench_name = comm_graph_file;
            int sz_sqrt = sqrt(sz);
            vector<point> partition_window;
            partition_window.push_back(point(2));
            partition_window[0][0] = std::stoi(milp_config.get_param("w0", "4"));
            partition_window[0][1] = std::stoi(milp_config.get_param("w1", "4"));
            partition_window.push_back(point(2));
            partition_window[1][0] = std::stoi(milp_config.get_param("w2", "2"));
            partition_window[1][1] = std::stoi(milp_config.get_param("w3", "2"));

            point graph_dims(2);
            graph_dims.set(sz_sqrt); // initial graph dimensions: we visualize as a mesh

            bool skip_cols = false;
            float wt_thresh = 1e5;
            conc_factor = 1;
            no_conc = 0;

            if (no_conc == true)
            { // If no_conc == true: Setup special dummy graph/nodes.
                hier_graphs.push_back(new Graph(hier_graphs.size()));
                hier_nodes.resize(hier_nodes.size() + 1);
                dims.div(2);
            }

            read_flows(bench_name, conc_factor, skip_cols, wt_thresh);
            if (no_conc == true)
            {
                hier_graphs[0] = hier_graphs[1];
                hier_nodes[0] = hier_nodes[1];
            }

            int n_levels = 2;

            max_dim = 4;
            dims.n = 4;
            dims.set(max_dim);

            for (int i = no_conc; i < n_levels; i++)
            {
                float intra_cluster_sum, inter_cluster_sum;

                Graph *graph_up = hier_graphs[i]->collapse_graph(dims, intra_cluster_sum, inter_cluster_sum, partition_window[i], graph_dims); // new graph at next/upper level in the hierarchy

                hier_graphs.push_back(graph_up);

                // cout << "At level " << i + 1 << ": intra-cluster sum " << intra_cluster_sum << ", inter-cluster sum " << inter_cluster_sum << endl;

                dims.div(2); // after collapsing, the graph dimensions are halved

                graph_dims[0] = graph_dims[0] / partition_window[i][0];
                graph_dims[1] = graph_dims[1] / partition_window[i][1];
            }

            std::cout << "Debug 3" << std::endl;

            // Hilbert h;
            // h.generateHilbert(dims.n, n_levels); // internally modifies h.mapping variable, which is used below
            dims.set(max_dim);
            // float hilbert_MCL = evaluate_mapping(hier_graphs[1]->adj, dims, h.mapping, 0); // hilbert curve mapping, preserves locality better than direct mapping
            // write_mapping(".hlb", h.mapping, dims);

            // float direct_MCL = direct_mapping(dims); // direct (naive) mapping
            float HTM_MCL = map_graph(); // RAHTM mapping (Phase 2 + 3)
#else
            std::cerr << "Error: RAHTM is only supported on Unix-like operating systems." << std::endl;
#endif
        }
        else
        {
            std::cerr << "Error: Unsupported algorithm ID: " << algorithm_id << std::endl;
            return 1;
        }
        topology->reset_topology();
        return 0;
        // ---------- END OF MILP SECTION ----------
    }
    // ---------- START OF ROUTING ----------
    std::vector<std::tuple<int, int, double>> routing_requests;
    routing_requests = aggregate_communication(comm_matrix, mapping);
    // Initialize the router
    std::vector<std::vector<Router *>> routers(num_iterations, std::vector<Router *>(num_vcs, nullptr));
    for (int iter = 0; iter < num_iterations; iter++)
    {
        for (int vc = 0; vc < num_vcs; ++vc)
        {
            if (vc_routing_algorithm[iter][vc] == "dimension_order_xy")
            {
                routers[iter][vc] = new DimensionOrder(topology, true);
            }
            else if (vc_routing_algorithm[iter][vc] == "dimension_order_yx")
            {
                routers[iter][vc] = new DimensionOrder(topology, false);
            }
            else if (vc_routing_algorithm[iter][vc] == "variable_dimension_order")
            {
                // routers[iter][vc] = new VariableDimensionOrder(topology);
            }
            else if (vc_routing_algorithm[iter][vc] == "destination_tag")
            {
                routers[iter][vc] = new DestinationTag(topology);
            }
            else if (vc_routing_algorithm[iter][vc] == "xor_tag")
            {
                routers[iter][vc] = new XORTag(topology);
            }
            else if (vc_routing_algorithm[iter][vc] == "valiant_xy")
            {
                routers[iter][vc] = new Valiant(topology, true);
            }
            else if (vc_routing_algorithm[iter][vc] == "valiant_yx")
            {
                routers[iter][vc] = new Valiant(topology, false);
            }
            else if (vc_routing_algorithm[iter][vc] == "minimal_or")
            {
                routers[iter][vc] = new MinimalOblivious(topology);
            }
            else if (vc_routing_algorithm[iter][vc] == "lbor")
            {
                routers[iter][vc] = new LoadBalancedOblivious(topology);
            }
            else
            {
                std::cerr << "Error: Unsupported routing algorithm for VC " << vc << std::endl;
                return 1;
            }
        }
    }
    try
    {
        // write the statistics from terminal in a file
        std::ofstream log_file("app/output/log.txt");
        if (!log_file.is_open())
        {
            std::cerr << "Error: Unable to open log.txt for writing." << std::endl;
            return 1;
        }
        std::ofstream routing_file("app/output/routing_paths.txt");
        if (!routing_file.is_open())
        {
            std::cerr << "Error: Unable to open routing_paths.txt for writing." << std::endl;
            return 1;
        }
        std::vector<pair<double, std::vector<int>>> routing_paths;
        for (int iter = 0; iter < num_iterations; iter++)
        {
            int current_vc = 0; // Start with VC 0
            double total_power_per_route = 0;
            double total_power_per_routes = 0;
            double total_power = 0;
            int total_hopcount = 0;
            int skips = 0;
            int num_faults = 0;
            // --- per-VC statistics ---
            std::vector<double> vc_total_power(num_vcs, 0.0);
            std::vector<int> vc_total_hopcount(num_vcs, 0);
            std::vector<int> vc_route_count(num_vcs, 0);
            // Print the iteration start
            std::cout << "-----------Start Of Iteration " << std::to_string(iter) << "----------------- \n";
            log_file << "-----------Start Of Iteration " << std::to_string(iter) << "----------------- \n";
            std::cout << "Routing Algorithm: " << vc_routing_algorithm[iter][0] << std::endl;
            log_file << "Routing Algorithm: " << vc_routing_algorithm[iter][0] << std::endl;
            
            for (const auto &[source, destination, flow] : routing_requests)
            {
                if (flow == 0)
                {
                    skips++;
                    continue;
                }
                std::cout << "Routing from Node " << source << " to Node " << destination << " with Flow: " << flow << " bits \n";
                log_file << "Routing from Node " << source << " to Node " << destination << " with Flow: " << flow << " bits \n";
                std::vector<int> path;
                int hopcount = 0;        
                // Assign VC in a round-robin manner
                int vc = current_vc;
                current_vc = (current_vc + 1) % num_vcs; // Move to the next VC
                // Perform routing
                path = routers[iter][vc]->route(source, destination, hopcount, vc, flow);
                if (path.empty())
                {
                    std::cerr << " Error: There is a faulty link in the path." << std::endl;
                    log_file << " Error: There is a faulty link in the path." << std::endl;
                    // skips++; as we already might route most of it before the faulty link is detected, we don't skip the flow here.
                    num_faults++;
                    continue;
                }
                if (path.front() != source)
                {
                    path.insert(path.begin(), source);
                    hopcount++;
                }
                routing_paths.push_back({flow, path});
                // Print the path
                for (int node : path)
                {
                    if (topology_name == "dragonfly")
                    {
                        if (node < routers_per_group * num_groups)
                        {
                            std::cout << "Router " << node << " -> ";
                            log_file << "Router " << node << " -> ";
                        }
                        else
                        {
                            // Print the node ID
                            std::cout << "Node " << node << " -> ";
                            log_file << "Node " << node << " -> ";
                        }
                    }
                    else
                    {
                        // Print the node ID
                        std::cout << "Node " << node << " -> ";
                        log_file << "Node " << node << " -> ";
                    }
                }
                std::cout << "Destination Reached!\n";
                log_file << "Destination Reached!\n";
                std::cout << "Hopcount: " << hopcount << std::endl;
                log_file << "Hopcount: " << hopcount << std::endl;
                std::cout << "VC Used: " << vc << std::endl;
                log_file << "VC Used: " << vc << std::endl;
                total_hopcount += hopcount;
                vc_total_hopcount[vc] += hopcount;
                vc_route_count[vc]++;
                
                if (power_model_mode == "detailed")
                {
                    // Calculate and print the total power for the route
                    total_power_per_route = calculate_detailed_power_for_route(path, flow, topology, link_capacitance, link_leakage_current,
                                                                               voltage, frequency, link_activity_factor,
                                                                               buffer_read_write_factor, crossbar_traversal_factor, buffer_capacitance, crossbar_capacitance, node_leakage_current);
                    total_power_per_routes += total_power_per_route;
                    vc_total_power[vc] += total_power_per_route;
                    std::cout << " Total Power (Nodes+Links) Due to This Route: " << total_power_per_route << std::endl;
                    log_file << " Total Power (Nodes+Links) Due to This Route: " << total_power_per_route << std::endl;
                }
                else if (power_model_mode == "none")
                {
                    std::cout << "Power model mode is set to none. Skipping power calculation." << std::endl;
                    log_file << "Power model mode is set to none. Skipping power calculation." << std::endl;
                }
                else
                {
                    std::cerr << "Error: Unsupported power model mode: " << power_model_mode << std::endl;
                    return 1;
                }
                 
            }
            std::cout << "-----------End Of Iteration " << std::to_string(iter) << "----------------- \n";
            log_file << "-----------End Of Iteration " << std::to_string(iter) << "----------------- \n";
            // Print MCL
            std::cout << "MCL: " << get_max_load(topology) / (normalization_factor) << " (GB) " << std::endl;
            log_file << "MCL: " << get_max_load(topology) / (normalization_factor) << " (GB) " << std::endl;
            if (vc_operation_mode == "round_robin")
            {
                total_power = calculate_total_power(topology);
                std::cout << "Total Power (Nodes+Links) :" << total_power << std::endl;
                std::cout << "Total Faulty Paths: " << num_faults << std::endl;
                log_file << "Total Power (Nodes+Links) :" << total_power << std::endl;
                log_file << "Total Faulty Paths: " << num_faults << std::endl;
                // Write routing file in this format ( to be used for simulator integration)
                routing_file << "## Metrics\n";
                routing_file << "MCL = " << get_max_load(topology) / (normalization_factor) << "\n";
                routing_file << "Total Hop Count = " << total_hopcount << "\n";
                routing_file << "Power Consumption (W) = " << total_power << "\n\n";
                // Print mapping in the following format
                routing_file << "## Thread Mapping" << "\n";
                routing_file << "Thread ID: ";
                for (int i = 0; i < (int)mapping.size(); ++i)
                {
                    routing_file << i << (i + 1 < (int)mapping.size() ? " " : "");
                }
                routing_file << "\n";
                routing_file << "Vertex ID: ";
                for (int i = 0; i < (int)mapping.size(); ++i)
                {
                    routing_file << mapping[i] << (i + 1 < (int)mapping.size() ? " " : "");
                }
                routing_file << "\n";
                routing_file << "## Routes\n";
                for (const auto &route : routing_paths)
                {
                    for (int i = 0; i < (int)route.second.size(); ++i)
                    {
                        routing_file << route.second[i];
                        if (i + 1 < route.second.size())
                            routing_file << " ";
                    }
                    routing_file << "\n";
                    routing_file << route.first << "\n"; // Write the flow size
                }
                // Write results to the CSV file
                csv_file << topology_name << ","
                         << vc_routing_algorithm[iter][0] << ","
                         << static_cast<double>(total_hopcount) << ","
                         << static_cast<double>(total_hopcount) / (routing_requests.size() - skips) << ","
                         << static_cast<double>(total_power_per_routes) / (routing_requests.size() - skips) << ","
                         << (get_max_load(topology) / normalization_factor) << ","
                         << total_power << "\n";
            }
            else if (vc_operation_mode == "custom")
            {
                // Map: algorithm name -> (total_power, total_hopcount, route_count,max_load)
                std::map<std::string, std::tuple<double, int, int>> algo_stats;
                // Build a map from algorithm name to the list of VCs using it
                std::map<std::string, std::vector<int>> algo_vcs;
                for (int vc = 0; vc < num_vcs; ++vc)
                {
                    algo_vcs[vc_routing_algorithm[iter][vc]].push_back(vc);
                }
                for (int vc = 0; vc < num_vcs; ++vc)
                {
                    const std::string &algo = vc_routing_algorithm[iter][vc];
                    double power = vc_total_power[vc];
                    int hops = vc_total_hopcount[vc];
                    int count = vc_route_count[vc];

                    auto &entry = algo_stats[algo];
                    std::get<0>(entry) += power;
                    std::get<1>(entry) += hops;
                    std::get<2>(entry) += count;
                }
                std::map<std::string, double> algo_max_link_load;
                const std::vector<Link *> &links = topology->get_all_links();
                for (const auto &[algo, vcs] : algo_vcs)
                {
                    double max_link_sum = 0.0;
                    for (Link *link : links)
                    {
                        if (!link)
                            continue;
                        double link_sum = 0.0;
                        for (int vc : vcs)
                        {
                            link_sum += link->get_max_load_per_vc_historical(vc);
                        }
                        if (link_sum > max_link_sum)
                        {
                            max_link_sum = link_sum;
                        }
                    }
                    algo_max_link_load[algo] = max_link_sum;
                }
                // Write per-algorithm results to CSV
                for (const auto &[algo, stats] : algo_stats)
                {
                    double total_power = std::get<0>(stats);
                    int total_hops = std::get<1>(stats);
                    int total_count = std::get<2>(stats);

                    double avg_hop = (total_count > 0) ? static_cast<double>(total_hops) / total_count : 0.0;
                    double avg_power = (total_count > 0) ? total_power / total_count : 0.0;

                    csv_file << topology_name << ","
                             << algo << ","
                             << total_hops << ","
                             << avg_hop << ","
                             << avg_power << ","
                             << algo_max_link_load[algo] / normalization_factor << ","
                             << total_power << "\n";
                }
            }
            topology->reset_topology(); // Reset the topology for the next iteration
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    // Close the CSV file
    csv_file.close();
    // Clean up
    delete topology;

    // delete mapper;
    for (int i = 0; i < num_iterations; i++)
    {

        for (int vc = 0; vc < num_vcs; ++vc)
        {
            delete routers[i][vc];
        }
    }
    // delete router;
    return 0;
}
