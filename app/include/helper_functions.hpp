#include <vector>
#include <map>
#include <tuple>
std::vector<std::tuple<int, int, double>> aggregate_communication(
    const std::vector<std::vector<double>> &comm_matrix,
    const std::vector<int> &mapping)
{
    int n = mapping.size();
    std::map<std::pair<int, int>, double> agg;

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i == j)
                continue;
            int phys_i = mapping[i];
            int phys_j = mapping[j];
            if (phys_i == phys_j)
                continue;
            std::pair<int, int> edge = {phys_i, phys_j};
            agg[edge] += comm_matrix[i][j];
        }
    }

    std::vector<std::tuple<int, int, double>> aggregated;
    for (auto &kv : agg)
    {
        aggregated.push_back(std::make_tuple(kv.first.first, kv.first.second, kv.second));
    }

    return aggregated;
}
// takes a comm_graph  and returns a comm_matrix
std::vector<std::vector<double>> comm_graph_to_matrix(
    const std::multimap<int, std::pair<int, int>>& communication_graph,
    int num_nodes)
{
    std::vector<std::vector<double>> comm_matrix(num_nodes, std::vector<double>(num_nodes, 0.0));
    for (const auto& entry : communication_graph) {
        int src = entry.first;
        int dst = entry.second.first;
        double value = static_cast<double>(entry.second.second);
        if (src >= 0 && src < num_nodes && dst >= 0 && dst < num_nodes) {
            comm_matrix[src][dst] += value;
        }
    }
    return comm_matrix;
}