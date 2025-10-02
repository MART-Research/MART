#ifndef TORUS_HELPER_HPP
#define TORUS_HELPER_HPP

#include <vector>
#include <tuple>
#include <random>
#include <algorithm>

// Helper function for Torus: get a random node in the quadrant between source and destination
int get_Random_Node_In_Quadrant_Helper(const std::vector<int> &src_coords, const std::vector<int> &dest_coords, int cols, int depth, std::mt19937 &rng);

#endif // TORUS_HELPER_HPP
