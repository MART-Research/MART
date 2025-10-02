#include "TorusHelper.hpp"

int get_Random_Node_In_Quadrant_Helper(const std::vector<int> &src_coords, const std::vector<int> &dest_coords, int cols, int depth, std::mt19937 &rng)
{
    if (depth == 1)
    {
        int min_row = std::min(src_coords[0], dest_coords[0]);
        int max_row = std::max(src_coords[0], dest_coords[0]);
        int min_col = std::min(src_coords[1], dest_coords[1]);
        int max_col = std::max(src_coords[1], dest_coords[1]);
        std::vector<std::pair<int, int>> candidates;
        int src_id = src_coords[0] * cols + src_coords[1];
        int dest_id = dest_coords[0] * cols + dest_coords[1];
        for (int row = min_row; row <= max_row; ++row)
        {
            for (int col = min_col; col <= max_col; ++col)
            {
                int node_id = row * cols + col;
                if (node_id != src_id && node_id != dest_id)
                {
                    candidates.emplace_back(row, col);
                }
            }
        }
        if (candidates.empty())
        {
            return src_id;
        }
        std::uniform_int_distribution<> idx_dist(0, candidates.size() - 1);
        auto [rand_row, rand_col] = candidates[idx_dist(rng)];
        return rand_row * cols + rand_col;
    }
    else
    {
        int min_row = std::min(src_coords[0], dest_coords[0]);
        int max_row = std::max(src_coords[0], dest_coords[0]);
        int min_col = std::min(src_coords[1], dest_coords[1]);
        int max_col = std::max(src_coords[1], dest_coords[1]);
        int min_depth = std::min(src_coords[2], dest_coords[2]);
        int max_depth = std::max(src_coords[2], dest_coords[2]);
        std::vector<std::tuple<int, int, int>> candidates;
        int src_id = src_coords[0] * cols * depth + src_coords[1] * depth + src_coords[2];
        int dest_id = dest_coords[0] * cols * depth + dest_coords[1] * depth + dest_coords[2];
        for (int row = min_row; row <= max_row; ++row)
        {
            for (int col = min_col; col <= max_col; ++col)
            {
                for (int dep = min_depth; dep <= max_depth; ++dep)
                {
                    int node_id = row * cols * depth + col * depth + dep;
                    if (node_id != src_id && node_id != dest_id)
                    {
                        candidates.emplace_back(row, col, dep);
                    }
                }
            }
        }
        if (candidates.empty())
        {
            return src_id;
        }
        std::uniform_int_distribution<> idx_dist(0, candidates.size() - 1);
        auto [rand_row, rand_col, rand_dep] = candidates[idx_dist(rng)];
        return rand_row * cols * depth + rand_col * depth + rand_dep;
    }
}
