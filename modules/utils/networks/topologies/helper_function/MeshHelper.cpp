#include "MeshHelper.hpp"

int get_Random_Node_In_Quadrant_Helper_Mesh(const std::vector<int> &src_coords, const std::vector<int> &dest_coords, int cols, std::mt19937 &rng)
{
    int min_row = std::min(src_coords[0], dest_coords[0]);
    int max_row = std::max(src_coords[0], dest_coords[0]);
    int min_col = std::min(src_coords[1], dest_coords[1]);
    int max_col = std::max(src_coords[1], dest_coords[1]);

    std::uniform_int_distribution<> row_dist(min_row, max_row);
    std::uniform_int_distribution<> col_dist(min_col, max_col);

    int rand_row = row_dist(rng);
    int rand_col = col_dist(rng);

    return rand_row * cols + rand_col;
}
