#ifndef MESH_HELPER_HPP
#define MESH_HELPER_HPP

#include <vector>
#include <random>
#include <algorithm>

// Helper function for Mesh: get a random node in the quadrant between source and destination
int get_Random_Node_In_Quadrant_Helper_Mesh(const std::vector<int> &src_coords, const std::vector<int> &dest_coords, int cols, std::mt19937 &rng);

#endif // MESH_HELPER_HPP
