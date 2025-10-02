#include "RingHelper.hpp"

int get_Random_Node_In_Quadrant_Helper_Ring(const std::vector<int> &src_coords, const std::vector<int> &dest_coords, int /*num_nodes*/, std::mt19937 &rng)
{
    int min_col = std::min(src_coords[1], dest_coords[1]);
    int max_col = std::max(src_coords[1], dest_coords[1]);
    std::uniform_int_distribution<> dist(min_col, max_col);
    int rand_col = min_col;
    while (rand_col == min_col || rand_col == max_col)
        rand_col = dist(rng);
    return rand_col;
}
