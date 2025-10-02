#ifndef RING_HELPER_HPP
#define RING_HELPER_HPP

#include <vector>
#include <random>
#include <algorithm>

// Helper function for Ring: get a random node in the quadrant between source and destination
int get_Random_Node_In_Quadrant_Helper_Ring(const std::vector<int> &src_coords, const std::vector<int> &dest_coords, int num_nodes, std::mt19937 &rng);

#endif // RING_HELPER_HPP
