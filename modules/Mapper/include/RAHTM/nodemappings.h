/*
The NodeMappings class provides a comprehensive set of tools for managing and manipulating mappings between nodes and 
multi-dimensional points. 
Its functions enable:
	Basic operations like rotation, merging, and adjustment of mappings.
	Advanced operations like evaluating mappings for quality and reusing patterns through analogy.
	Flexibility in handling hierarchical or graph-based data in a spatial context
*/
#ifndef MAPPING_H
#define MAPPING_H

#include <stdlib.h>
#include <vector>
#include <map>
#include <set>
#include <ostream>
#include <list>

#include "point.h"
#include "LP_nodemappings.h" // For the LP-based approach for mapping processes/threads to compute nodes

using namespace std;

/*
    The NodeMappings class manages mappings between node IDs and multi-dimensional points,
    providing various operations for rotating, merging, and evaluating these mappings.
*/
class NodeMappings {
public:
    float score;                    // Quality score of the current mapping
    map<long, point> mapping;      // Mapping from node IDs to their coordinates
    point dims;                    // Dimensions of the coordinate space
    point starting_point;          // Base point for coordinate adjustments
    int nDims;                     // Number of active dimensions
    long rotation_seq;             // Current rotation sequence number
    long max_rotations;            // Maximum possible rotations for this dimensionality
    int last_flip_dim;             // Last dimension that was flipped
    bool flip_dir;                 // Direction of flipping (true: X->Y->Z, false: Z->Y->X)

    // Comparison operator for sorting mappings by score
    bool operator() (const NodeMappings &nm1, const NodeMappings &nm2) { 
        return (nm1.score < nm2.score);
    }

    // Constructor - initializes rotation and flip state
    NodeMappings() {
        rotation_seq = 0;
        flip_dir = true;
        last_flip_dim = 0;  // Start with first dimension
        resetScore();
    }

    // Sets the dimensions of the coordinate space and calculates maximum rotations
    void setDims(point &dims) {
        this->dims = dims;
        nDims = 0;

        // Count dimensions with size > 1
        for(int d = 0; d < dims.n; d++)
            if(dims[d] > 1)
                nDims++;

        max_rotations = nDims * (1 << nDims);  // nDims * 2^nDims
    }

    // Resets the mapping score to a high default value
    void resetScore() { score = 1e10; }

    // Collects all node IDs from the current mapping into the set ids
    void getIds(set<long> &ids) {
        for(auto it = mapping.begin(); it != mapping.end(); it++)
            ids.insert(it->first);
    }

    // Rotates and potentially flips all points in the mapping
    void rotateMapping() {
        rotation_seq++;
        if(nDims == 1) { // Special case for 1D - simple flip
            assert(mapping.size() == 2);
            for(auto it = mapping.begin(); it != mapping.end(); it++) {
                it->second[0] = 1 - it->second[0];  // Flip between 0 and 1
            }
            return;
        }

        bool is_flip = (rotation_seq % nDims) == 0;  // Determine if this is a flip rotation
        for(auto it = mapping.begin(); it != mapping.end(); it++) {
            it->second.rotate_upper(nDims);	// Rotate coordinates
            if(is_flip)	// Flip coordinates in current dimension if needed
				it->second[last_flip_dim] = (dims[last_flip_dim] - 1) - it->second[last_flip_dim];
        }

        // Update flip state for next rotation
        if(is_flip) {
            last_flip_dim += flip_dir ? 1 : -1;
            if(last_flip_dim == 0) flip_dir = true;
            else if(last_flip_dim == (nDims - 1)) flip_dir = false;
        }
    }

    // Merges two mappings by placing nm1 in plane 0 and nm2 in plane 1 of a new dimension
    void mergeMappings(NodeMappings &nm1, NodeMappings &nm2) {
        mapping.clear();
        dims = nm1.dims;
        int d = nm1.nDims;
        dims[d] = 2;       // Add new dimension with size 2
        nDims = d + 1;     // Increment dimension count
        
        // Add nm1 points in plane 0
        for(auto it = nm1.mapping.begin(); it != nm1.mapping.end(); it++) {
            mapping[it->first] = it->second;
            mapping[it->first][d] = 0;
        }

        // Add nm2 points in plane 1
        for(auto it = nm2.mapping.begin(); it != nm2.mapping.end(); it++) {
            mapping[it->first] = it->second;
            mapping[it->first][d] = 1;
        }
    }

    // Adjusts all mappings by adding an offset point (no normalization)
    void adjustMappings(point &st) {
        for(auto it = mapping.begin(); it != mapping.end(); it++) 
			it->second.add(st);
    }

    // Adjusts all mappings by adding an offset point and normalizing within dimensions
    void adjustMappings(point &st, point &dims) {
        for(auto it = mapping.begin(); it != mapping.end(); it++) {
            it->second.add(st);
            it->second.normalize(dims);
        }
    }

    // Adds all mappings from another NodeMappings object
	// Non-destructive combination (preserves source mapping), Handles overlapping node IDs via overwrite
    void addMappings(NodeMappings &nm) {
        for(auto it = nm.mapping.begin(); it != nm.mapping.end(); it++)
            mapping[it->first] = it->second;
    }

    // Rotates and merges two mappings while evaluating for best MCL score
	// rotates nm2, merges with nm1, repeats to find the combination with the best MCL score
    void rotateAndMergeMapping(NodeMappings nm1, NodeMappings nm2, FlowSet &fs, bool is_mesh) {
        NodeMappings nm;
        float best_MCL = 1e10;
        do {
            nm.mergeMappings(nm1, nm2);
            float MCL = evaluate_mapping(fs, dims, nm.mapping, is_mesh);
            if(MCL < best_MCL) mapping = nm.mapping, best_MCL = MCL;
            nm2.rotateMapping();
        } while(nm2.rotation_seq < nm2.max_rotations);

        score = best_MCL;
    }

    // Creates a trivial sequential mapping of nodes to points, Assigns nodes to coordinates in lexicographical order
	// Provides fallback/default mapping configuration
    void triviallyMap(set<long> &nodes) {
        point p;
        for(auto n_it = nodes.begin(); n_it != nodes.end(); n_it++) {
            mapping[*n_it] = p;
            p.inc(dims);
        }
    }
    
	// Copies/reuses successful patterns between similar subgraphs, accelerates convergence by avoiding recomputations of similar subgraphs
    /*
    node_ids: flat set of node IDs to be mapped
    analogy_mapping: reference mapping to be used for filling
    It edits a class variable: mapping

    Used during top-down mapping (phase 2)
    It applies pre-computed mappings to new nodes at the same hierarchy level
    Example: Reusing a optimized 4-node mapping for another similar 4-node group
    */
    void fill_by_analogy(set<long> &node_ids, map<long, point> &analogy_mapping) {
        assert(node_ids.size() <= analogy_mapping.size());
        set<long>::iterator it1 = node_ids.begin();
        for(auto it2 = analogy_mapping.begin(); it1 != node_ids.end(); it1++, it2++)
			mapping[*it1] = it2->second;
    }
};

#endif