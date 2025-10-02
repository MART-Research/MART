#include "../include/MinimalOblivious.hpp"
#include <cstdlib>
#include <ctime>

MinimalOblivious::MinimalOblivious(Topology *topo) : Router(topo) {
    // Seed random number generator
    srand(static_cast<unsigned int>(time(nullptr)));
}

std::vector<std::vector<int>> MinimalOblivious::get_All_Minimal_paths(int source, int destination, std::vector<int> &hopcounts)
{
    std::vector<std::vector<int>> all_paths;
    generateMinimalPaths(source, destination, all_paths, hopcounts);
    return all_paths;
}

std::vector<int> MinimalOblivious::route_hypercube(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);

    std::vector<int> path;
    int current = source;
    hopcount = 0;
    path.push_back(current);
    if (current == destination)
    {
        return path;
    }
    while (current != destination)
    {
        int differing_bits = current ^ destination;
        int move_bit = 1;
        while ((differing_bits & move_bit) == 0)
        {
            move_bit <<= 1;
        }
        int next_node = current ^ move_bit;
        Link *link = topology->get_link(current, next_node);
        if (!link->getActive())
        {
            // Indicate a routing fault by returning an empty path
            return std::vector<int>();
        }
        link->update_vc_load(vc, flow);
        // link->update_max_load();
        path.push_back(current); // note it'll push the src
        current = next_node;
        hopcount++;
    }

    path.push_back(destination);
    // remove the src node from the path to be consistent with othre representations
    path.erase(path.begin());
    return path;
}

std::vector<int> MinimalOblivious::route_dragonfly(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);

    Dragonfly *dragonfly_topology = dynamic_cast<Dragonfly *>(topology);
    if (!dragonfly_topology)
    {
        throw std::runtime_error("Topology is not Dragonfly!");
    }

    std::vector<int> path;
    std::unordered_map<int, int> parent; // To reconstruct the path
    std::queue<int> bfs_queue;
    std::unordered_set<int> visited;

    bfs_queue.push(source);
    visited.insert(source);
    parent[source] = -1;

    // Perform BFS
    while (!bfs_queue.empty())
    {
        int current = bfs_queue.front();
        bfs_queue.pop();

        if (current == destination)
        {
            break;
        }

        for (int neighbor : dragonfly_topology->get_nodes_list()[current]->get_neighbors())
        {
            if (visited.find(neighbor) == visited.end())
            {
                visited.insert(neighbor);
                parent[neighbor] = current;
                bfs_queue.push(neighbor);
            }
        }
    }

    // Reconstruct the path
    int current = destination;
    while (current != -1)
    {
        path.push_back(current);
        current = parent[current];
    }
    std::reverse(path.begin(), path.end());

    // Update hopcount and link loads
    hopcount = path.size() - 1;
    for (int i = 0; i < path.size() - 1; i++)
    {
        Link *link = topology->get_link(path[i], path[i + 1]);
        if (!link->getActive())
        {
            // Indicate a routing fault by returning an empty path
            return std::vector<int>();
        }

        link->update_vc_load(vc, flow);
        // link->update_max_load();
    }

    return path;
}

std::vector<int> MinimalOblivious::route_butterfly(int source, int destination, int &hopcount, int vc, double flow, int node)
{
    validate_node(source);
    validate_node(destination);
    // cout << "Routing from Node " << source << " to Node " << destination << ":\n";
    if (source == destination)
    {
        vector<int> path;
        path.push_back(source);
        return path;
    }
    std::vector<int> shortest_path;
    int min_hops = 100000;
    std::vector<int> temp_path;
    std::unordered_set<int> visited;
    int temp_hopcount = 0;

    backtrack(source, destination, temp_path, shortest_path,
              visited, temp_hopcount, vc, flow, min_hops, node);

    hopcount = shortest_path.empty() ? 0 : shortest_path.size() - 1;

    for (int i = 0; i < shortest_path.size() - 1; i++)
    {
        Link *link = topology->get_link(shortest_path[i], shortest_path[i + 1]);
        if (!link->getActive())
        {
            // Indicate a routing fault by returning an empty path
            return std::vector<int>();
        }
        link->update_vc_load(vc, flow);
        // link->update_max_load();
    }
    return shortest_path;
}
void MinimalOblivious::backtrack(int current_node, int destination, std::vector<int> &temp_path,
                                 std::vector<int> &shortest_path, std::unordered_set<int> &visited,
                                 int &temp_hopcount, int vc, double flow, int &min_hops, int /*node*/)
{
    temp_path.push_back(current_node);
    visited.insert(current_node);

    if (current_node == destination)
    {
        if (temp_path.size() < min_hops)
        {
            shortest_path = temp_path;
            min_hops = temp_path.size();
        }
        visited.erase(current_node);
        temp_path.pop_back();
        return;
    }

    if (temp_path.size() >= min_hops)
    {
        visited.erase(current_node);
        temp_path.pop_back();
        return;
    }

    vector<int> neighbors = topology->get_nodes_list()[current_node]->get_neighbors();
    for (int neighbor : neighbors)
    {
        if (visited.find(neighbor) == visited.end())
        {
            temp_hopcount++;

            backtrack(neighbor, destination, temp_path, shortest_path,
                      visited, temp_hopcount, vc, flow, min_hops);
            temp_hopcount--;
        }
    }

    // Backtrack
    visited.erase(current_node);
    temp_path.pop_back();
}

std::vector<int> MinimalOblivious::route(int source, int destination, int &hopcount, int vc, double flow)
{
    validate_node(source);
    validate_node(destination);
    if (topology->get_topology_name() == "Tree")
    {
        throw std::runtime_error("Tree topology is not supported by MinimalOblivious routing ."); // As there is only one minimal path so same
    }
    if (topology->get_topology_name() == "Hypercube")
    {
        return route_hypercube(source, destination, hopcount, vc, flow);
    }
    if (topology->get_topology_name() == "Dragonfly")
    {
        return route_dragonfly(source, destination, hopcount, vc, flow);
    }
    if (topology->get_topology_name() == "Butterfly")
    {
        return route_butterfly(source, destination, hopcount, vc, flow);
    }
    std::vector<int> hopcounts;
    // Retrieve all minimal paths
    vector<vector<int>> all_paths = get_All_Minimal_paths(source, destination, hopcounts);

    // Ensure at least one path exists
    if (all_paths.empty())
    {
        std::cerr << "No minimal path without faults found from " << source << " to " << destination << std::endl;
        return std::vector<int>();
    }
    
    // // Debug: Print number of minimal paths found
    // std::cout << "Found " << all_paths.size() << " minimal paths from " << source << " to " << destination << std::endl;
    // for (int i = 0; i < all_paths.size(); i++) {  // Print first 3 paths
    //     std::cout << "Path " << i << ": ";
    //     for (int node : all_paths[i]) {
    //         std::cout << node << " ";
    //     }
    //     std::cout << "(hops: " << hopcounts[i] << ")" << std::endl;
    // }
    
    // Randomly select a path from all minimal paths
    int path_index = rand() % all_paths.size();
    // std::cout << "Selected path index: " << path_index << std::endl;
    std::vector<int> selected_path = all_paths[path_index];
    int current_node;
    int next_node;
    for (int i = 0; i < selected_path.size() - 1; i++)
    {
        current_node = selected_path[i];
        next_node = selected_path[i + 1];
        Link *link = topology->get_link(current_node, next_node);
        if (!link->getActive())
        {
            // Indicate a routing fault by returning an empty path
            return std::vector<int>();
        }
        link->update_vc_load(vc, flow);
        // link->update_max_load();
    }
    hopcount = hopcounts[path_index];
    return selected_path;
}
void MinimalOblivious::generateMinimalPaths(int source, int destination, std::vector<std::vector<int>> &all_paths, std::vector<int> &hopcounts)
{
    int rows = topology->get_num_rows();
    int cols = topology->get_num_cols();
    int depths = topology->get_depth();
    std::string topo_name = topology->get_topology_name();
    bool is_ring = (topo_name == "Ring");
    if (is_ring)
    {
        rows = 1;
        cols = topology->get_num_nodes();
    }

    // Calculate source and destination coordinates
    int src_row = source / (cols * depths);
    int src_col = (source / depths) % cols;
    int src_depth = source % depths;
    int dest_row = destination / (cols * depths);
    int dest_col = (destination / depths) % cols;
    int dest_depth = destination % depths;

    // Determine if topology is torus
    bool is_torus2d = (topology->get_topology_name() == "Torus_2D");
    bool is_torus3d = (topology->get_topology_name() == "Torus_3D");

    // Compute minimal moves for each dimension (with wrap-around for torus)
    std::vector<std::vector<std::pair<char, int>>> all_move_sets; // Multiple move sets for equal distance cases
    std::vector<std::pair<char, int>> current_moves; // Current move set being built
    
    auto add_moves = [&](int src, int dest, int dim, char pos, char neg, bool is_torus)
    {
        int direct = dest - src;
        int size = dim;
        if (is_torus)
        {
            int forward = (dest - src + size) % size;
            int backward = (src - dest + size) % size;
            if (forward < backward)
            {
                current_moves.emplace_back(pos, forward);
            }
            else if (backward < forward)
            {
                current_moves.emplace_back(neg, backward);
            }
            else if (forward == backward && forward != 0)
            { // both are minimal, need to generate separate move sets
                // printf("Both directions are minimal: %d %d\n", forward, backward);
                if (all_move_sets.empty()) {
                    // First equal case - create two sets from current moves
                    std::vector<std::pair<char, int>> moves_pos = current_moves;
                    std::vector<std::pair<char, int>> moves_neg = current_moves;
                    moves_pos.emplace_back(pos, forward);
                    moves_neg.emplace_back(neg, backward);
                    all_move_sets.push_back(moves_pos);
                    all_move_sets.push_back(moves_neg);
                    current_moves.clear(); // Clear since we're now using all_move_sets
                } else {
                    // Already have multiple sets - duplicate each and add both directions
                    std::vector<std::vector<std::pair<char, int>>> new_sets;
                    for (const auto& move_set : all_move_sets) {
                        std::vector<std::pair<char, int>> set_pos = move_set;
                        std::vector<std::pair<char, int>> set_neg = move_set;
                        set_pos.emplace_back(pos, forward);
                        set_neg.emplace_back(neg, backward);
                        new_sets.push_back(set_pos);
                        new_sets.push_back(set_neg);
                    }
                    all_move_sets = new_sets;
                }
            }
        }
        else
        {
            if (direct > 0) {
                if (all_move_sets.empty()) {
                    current_moves.emplace_back(pos, direct);
                } else {
                    for (auto& move_set : all_move_sets) {
                        move_set.emplace_back(pos, direct);
                    }
                }
            } else if (direct < 0) {
                if (all_move_sets.empty()) {
                    current_moves.emplace_back(neg, -direct);
                } else {
                    for (auto& move_set : all_move_sets) {
                        move_set.emplace_back(neg, -direct);
                    }
                }
            }
        }
    };

    if (is_ring)
    {
        // Only R/L moves for ring, always wrap-around
        add_moves(src_col, dest_col, cols, 'R', 'L', true);
        // Do NOT add D/U/I/O moves for ring
    }
    else
    {
        // X (cols)
        add_moves(src_col, dest_col, cols, 'R', 'L', is_torus2d || is_torus3d);
        // Y (rows)
        add_moves(src_row, dest_row, rows, 'D', 'U', is_torus2d || is_torus3d);
        // Z (depths, only for 3D)
        if (depths > 1)
            add_moves(src_depth, dest_depth, depths, 'I', 'O', is_torus3d);
    }

    // Finalize move sets
    if (all_move_sets.empty() && !current_moves.empty()) {
        all_move_sets.push_back(current_moves);
    }

    // Build all move strings for each move set
    std::vector<std::string> all_move_strings;
    for (const auto& moves : all_move_sets) {
        std::vector<std::string> move_strings{""};
        for (auto &[move, count] : moves)
        {
            std::vector<std::string> new_strings;
            for (auto &s : move_strings)
            {
                std::string m(count, move);
                new_strings.push_back(s + m);
            }
            move_strings = new_strings;
        }
        all_move_strings.insert(all_move_strings.end(), move_strings.begin(), move_strings.end());
    }

    // For each move string, generate all unique permutations and validate
    std::unordered_set<std::string> unique_perms;
    for (auto &move_str : all_move_strings)
    {
        std::string path = move_str;
        std::sort(path.begin(), path.end());
        do
        {
            if (unique_perms.count(path))
                continue;
            unique_perms.insert(path);
            bool valid_path = true;
            int current_node = source;
            std::vector<int> path_moves{current_node};
            int hopcount = 0;
            for (char move : path)
            {
                int next_node = getNextNode(current_node, move, cols, depths);
                try
                {
                    topology->get_link(current_node, next_node);
                }
                catch (const std::out_of_range &)
                {
                    valid_path = false;
                    break;
                }
                current_node = next_node;
                path_moves.push_back(current_node);
                hopcount++;
            }
            if (valid_path)
            {
                all_paths.push_back(path_moves);
                hopcounts.push_back(hopcount);
            }
        } while (std::next_permutation(path.begin(), path.end()));
    }
}

int MinimalOblivious::getNextNode(int current_node, char move, int cols, int depths)
{
    int rows = topology->get_num_rows();
    std::string topo_name = topology->get_topology_name();
    bool is_torus2d = (topo_name == "Torus_2D");
    bool is_torus3d = (topo_name == "Torus_3D");
    bool is_ring = (topo_name == "Ring");
    if (is_ring)
    {
        // printf("Ring topology detected\n");
        // Special case: 1D Ring
        int num_nodes = topology->get_num_nodes();
        int next = current_node;
        switch (move)
        {
        case 'R':
            next = (current_node + 1) % num_nodes;
            break;
        case 'L':
            next = (current_node - 1 + num_nodes) % num_nodes;
            break;
        default:
            throw std::invalid_argument("Invalid move for Ring");
        }
        return next;
    }
    else if (depths == 1)
    {
        // 2D case (mesh, torus2d)
        int row = current_node / cols;
        int col = current_node % cols;
        switch (move)
        {
        case 'R':
            if (is_torus2d)
                col = (col + 1) % cols;
            else
                col = col + 1;
            break;
        case 'L':
            if (is_torus2d)
                col = (col - 1 + cols) % cols;
            else
                col = col - 1;
            break;
        case 'D':
            if (is_torus2d)
                row = (row + 1) % rows;
            else
                row = row + 1;
            break;
        case 'U':
            if (is_torus2d)
                row = (row - 1 + rows) % rows;
            else
                row = row - 1;
            break;
        default:
            throw std::invalid_argument("Invalid move");
        }
        return row * cols + col;
    }
    else
    {
        // 3D case (torus3d)
        int row = current_node / (cols * depths);
        int col = (current_node / depths) % cols;
        int dep = current_node % depths;
        switch (move)
        {
        case 'R':
            if (is_torus3d)
                col = (col + 1) % cols;
            else
                col = col + 1;
            break;
        case 'L':
            if (is_torus3d)
                col = (col - 1 + cols) % cols;
            else
                col = col - 1;
            break;
        case 'D':
            if (is_torus3d)
                row = (row + 1) % rows;
            else
                row = row + 1;
            break;
        case 'U':
            if (is_torus3d)
                row = (row - 1 + rows) % rows;
            else
                row = row - 1;
            break;
        case 'I':
            if (is_torus3d)
                dep = (dep + 1) % depths;
            else
                dep = dep + 1;
            break;
        case 'O':
            if (is_torus3d)
                dep = (dep - 1 + depths) % depths;
            else
                dep = dep - 1;
            break;
        default:
            throw std::invalid_argument("Invalid move");
        }
        return row * cols * depths + col * depths + dep;
    }
}