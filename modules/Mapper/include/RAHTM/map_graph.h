#ifndef MAP_GRAPH_H
#define MAP_GRAPH_H

#include <vector>
#include <map>
#include <set>
#include "node_rahtm.h"
#include "graph.h"
#include "nodemappings.h"
#include "heurnodemappings.h"
#include "IMR_mapping.h"
#include "hypercubepartition.h"
#include "fattreepartition.h"
#include "dragonflypartition.h"


#define BEAM_WIDTH_1 32
#define BEAM_WIDTH_2 16

using namespace std;

extern vector<map<long, Node_rahtm*> > hier_nodes;
extern vector<NodeMappings> hier_mapping;
extern vector<Graph*> hier_graphs;
extern int max_dim;
extern string bench_name;
extern int conc_factor;
extern bool no_conc;


#define FILTER_ROTATIONS 1 // Enable rotation optimization
#define USE_ANALOGY_HYPERCUBES 1 // reuse successful patterns
#define IS_IMR 1 // Use OPTIMAR MILP formulation for mapping

/*

Copies successful patterns between similar subgraphs
used for accelerating convergence of the mapping algorithm

nodes: set of parent Node_rahtm objects
analogy_mapping: reference mapping
nm: mapping object to be filled

Handles multi-level node relationships
Used during bottom-up merging (Phase 3)
Propagates mappings upward through the hierarchy
Example: When merging two 4-node blocks, reuse a known good mapping for their combined 8-node structure
*/
void fill_by_analogy(set<Node_rahtm*> &nodes, map<long, point> &analogy_mapping, NodeMappings &nm) {
	set<long> node_ids;
	for(auto it1 = nodes.begin(); it1 != nodes.end(); it1++) {
		set<Node_rahtm*> nodes2; (*it1)->get_set_children(nodes2); // retrive the child nodes of each nodes element, and save into nodes2
		for(auto it2 = nodes2.begin(); it2 != nodes2.end(); it2++) node_ids.insert((*it2)->id); // insert the IDs of the child nodes into node_ids
	}
	
	assert(node_ids.size() <= analogy_mapping.size());
	set<long>::iterator it1 = node_ids.begin();
	map<long, point>::iterator it2 = analogy_mapping.begin();
	for(; it1 != node_ids.end(); it1++, it2++)
		nm.mapping[*it1] = it2->second;
}

/*
Outputs mapping to file
*/
void write_mapping(string ext, map<long, point> mapping, point &dims)
{
	stringstream file_name;
	size_t lastSlash = bench_name.find_last_of("/");
	string outputfilename = (lastSlash != std::string::npos) ? bench_name.substr(lastSlash + 1) : bench_name;
	file_name << "app/output/" << outputfilename << "_"; dims.write_point(file_name); file_name << ext;

	vector<long> node_thread_count(mapping.rbegin()->first + 1, 0);
	ofstream fp_out(file_name.str().c_str(), ofstream::out);
	
	for(auto it = hier_graphs[no_conc]->node_partition_assign.begin(); it != hier_graphs[no_conc]->node_partition_assign.end(); it++) {
		mapping[it->second].write_point_special(fp_out);
		fp_out << " " << node_thread_count[it->second]++ << endl;
	}
	
	fp_out.close();
	
	if(ext.compare(".htm") == 0) {
		file_name << ".mapping";
		ofstream fp_map(file_name.str().c_str(), ofstream::out);
		
		for(auto map_it = mapping.begin(); map_it != mapping.end(); map_it++) {
			fp_map << map_it->first << " : ";
			map_it->second.write_point_special(fp_map);
			fp_map << endl;
		}
		fp_map.close();
	}
}

void adjust_mapping(map<long, point> &mapping, point starting_point, map<long, point> &adjusted_mapping, int scale)
{
	for(auto map_it = mapping.begin(); map_it != mapping.end(); map_it++) {
		adjusted_mapping[map_it->first] = starting_point;
		adjusted_mapping[map_it->first].mul(scale);
		adjusted_mapping[map_it->first].add(map_it->second);
	}
}

void recurse_map(set<long> &threads, map<point, bool> &locations, vector<map<long, point> > &mappings, bool is_root)
{
	if(threads.size() == 0)
		return;

	if(is_root) {
		mappings.push_back(map<long, point>());
	} 

	long thread = *threads.begin();
	threads.erase(threads.begin());

	for(auto loc_it = locations.begin(); loc_it != locations.end(); loc_it++) {
		if(!loc_it->second) {
			loc_it->second = true;
			
			(*mappings.rbegin())[thread] = loc_it->first;

			recurse_map(threads, locations, mappings, false);
			loc_it->second = false;
			mappings.push_back(*mappings.rbegin());
		}
	}

	mappings.pop_back();
	threads.insert(thread);
}

/*
Generates all possible task-node assignments
Used for: Exhaustive search in small cases
*/
void get_mappings(point &dims, set<Node_rahtm*> &nodes, vector<map<long, point> > &mappings)
{
	mappings.clear();
	map<point, bool> locations;
	for(int d1 = 0; d1 < dims[0]; d1++)
		for(int d2 = 0; d2 < dims[1]; d2++) {
			point p;
			p[0] = d1, p[1] = d2;
			locations[p] = false;
		}

	set<long> threads;
	for(auto node_it = nodes.begin(); node_it != nodes.end(); node_it++) threads.insert((*node_it)->id);
	recurse_map(threads, locations, mappings, true);
}

/*
propagates mapping through/down the hierarchy
analogy_mapping is copied into the mapping of the nodes at level = "level"
*/
void distribute_mapping(int level, const map<long, point>& analogy_mapping) {
    for (const auto& [id, mapping] : analogy_mapping) {
        Node_rahtm* node = hier_nodes[level][id];
        node->mapping = mapping;
    }
}

/*
	nodes: set of nodes to merge at current level
	level_step: step difference to reach children level
	dim: size of dimension grid
	parent_node: pointer to parent node (to collect merged mapping)
	is_mesh: whether the topology is a mesh
*/

float map_quads(set<Node_rahtm*> &nodes, int level, int level_step, int dim, Node_rahtm* parent_node, bool is_mesh) 
{
	point dims; dims.set(dim); // a point representing the full grid size at this level, we need dims to know how big the global space is

	point dims2; dims2.set(dim >> 1); // later used for adjusting children's local grids

	// relocate each cluster's local mapping into into the correct location inside the big global grid
	for(auto it1 = nodes.begin(); it1 != nodes.end(); it1++) {
		point st((*it1)->mapping);
		st.mul(1 << level_step);
		(*it1)->children_mapping.adjustMappings(st, dims);
	}
		
	set<Node_rahtm*> all_children_nodes;

	vector<HeurNodeMappings*> all_heurs; // represents the list of clusters we are trying to merge
	all_heurs.reserve(nodes.size());
	
	for(auto it1 = nodes.begin(); it1 != nodes.end(); it1++) {

		(*it1)->get_n_level_set_children(level_step, all_children_nodes);
		all_heurs.push_back(&(*it1)->children_mapping);

		if(FILTER_ROTATIONS) {
			set<Node_rahtm*>::iterator it2 = it1;
			for(it2++; it2 != nodes.end(); it2++) {
				int d = (*it1)->mapping.isNeighbor((*it2)->mapping);
				if(d >= 0) {
					set<Node_rahtm*> children_nodes;
					(*it1)->get_n_level_set_children(level_step, children_nodes);
					(*it2)->get_n_level_set_children(level_step, children_nodes);

					set<long> node_ids;
					for(auto nit = children_nodes.begin(); nit != children_nodes.end(); nit++) node_ids.insert((*nit)->id);

					FlowSet fs; hier_graphs[level]->adj.getSubset(node_ids, fs);

					HeurNodeMappings &h1 = (*it1)->children_mapping;
					HeurNodeMappings &h2 = (*it2)->children_mapping;

					h1.heurRotateAndMergeMapping(h2, fs, d, is_mesh);
				}
			}
		}
	}

	multimap<float, HeurNodeMappings*, std::greater<float> > nodes_sorted;
	// key: MCL, value: pointer to HeruNodeMappings, contains clusters sorted desc., clusters with worst MCL first (to give them maximum flexibility)

	auto it = all_heurs.begin();
	for(int i = 0; it != all_heurs.end(); it++, i++) {
		if(FILTER_ROTATIONS) {
			vector<NodeMappings>::iterator rot_it = (*it)->all_rotations.begin(); // take the first rotation
			nodes_sorted.insert(pair<float, HeurNodeMappings*> (rot_it->score, *it)); // 
		} else
			nodes_sorted.insert(pair<float, HeurNodeMappings*> (i, *it));
	}

	assert(nodes_sorted.size() > 2); // contains all the blocks we are about to merge, sorted based on their initial mapping quality

	// ---------------------------------------- Merging of sub-problem mappings -----------------------------------
	// The idea is to merge the two worst clusters first, and then merge the rest of the clusters with the merged cluster
	// Remember: We try to combine the heavy traffic clusters first, giving them maximum flexibility
	auto nit1 = nodes_sorted.begin(); // nit1 points to the Node_rahtm/cluster with worst MCL
	auto nit2 = nit1; nit2++; // nit2 points to the Node_rahtm/cluster with 2nd worst MCL

	// nit1->second = the heuristic mapping of the first cluster, all_rotations is the multiple rotations within that cluster
	long beam_width = nit1->second->all_rotations.size();
	vector<NodeMappings> current_best_merged_mappings;
	vector<NodeMappings> temp_merged_mapping;
	vector<NodeMappings> *arm_p1, *arm_p2, *temp;
	arm_p1 = &current_best_merged_mappings;
	arm_p2 = &temp_merged_mapping;
	
	// get all node IDs of nit1's and nit2's HeurModeMappings and save them in node_ids
	set<long> node_ids;
	nit1->second->getIds(node_ids);
	nit2->second->getIds(node_ids);
	// now node_ids holds the IDs of the clusters to be merged

	FlowSet fs; hier_graphs[level]->adj.getSubset(node_ids, fs); // get the set of flows between the two clusters to be merged

	// Now, given all rotations of both clusters, find the top beam_width mergings (ones with lowest MCL) and save them to current_best_merged_mappings
	// This is similar to the concept of Beam Search, where we only keep the best beam_width number of solutions instead of exploring the full search space
	HeurNodeMappings::incrementalRotations(nit1->second->all_rotations, nit2->second->all_rotations, current_best_merged_mappings, fs, beam_width, dims, is_mesh);

	float MCL = -1;
	long iter_count = 0;
	for(nit2++; nit2 != nodes_sorted.end(); nit2++, iter_count++) {
		nit2->second->getIds(node_ids);

		FlowSet fs; hier_graphs[level]->adj.getSubset(node_ids, fs);

		MCL = HeurNodeMappings::incrementalRotations(*arm_p1,  nit2->second->all_rotations, *arm_p2, fs, beam_width, dims, is_mesh);
			
		arm_p1->clear();
		temp = arm_p1;
		arm_p1 = arm_p2;
		arm_p2 = temp;
		if(iter_count >= 8) {
			if(iter_count < 13)
				beam_width = BEAM_WIDTH_1;
			else 
				beam_width = BEAM_WIDTH_2;
		}
	}
	// ------------------------------------ UNDERSTOOD BUT NEEDS TO BE REFACTORED FOR READABILITY (SEE modifications.txt - RFCTR1) -------------------------------

	// after finishing the merging of all nodes, take the final best merged mapping, and write it back to into the actual node objects at this level
	distribute_mapping(level, arm_p1->begin()->mapping);

	// store the merged mapping into the parent node, so that when we go to merge parents later, each parent has its own children_mapping ready
	if(parent_node != NULL) {
		parent_node->children_mapping.fill(dims, *arm_p1->begin());
	}

	if(level == 1) { // the lowest non-trival level (right above the leaf nodes)
		float final_mcl = evaluate_mapping(hier_graphs[level]->adj, dims, arm_p1->begin()->mapping, 0);
		cout << "----CONFIRM: " << final_mcl << endl;
		MCL = final_mcl;
	}
	
	return MCL;
}

/*
Recursively processes hierarchy levels:
	Processes nodes level-by-level
	Uses map_quads() for local optimization
	Propagates mappings upward
*/
float recurse_quads2(int level, int level_step, set<Node_rahtm*> nodes, int dim, Node_rahtm* parent_node, bool is_mesh)
{
	point dims; dims.set(dim); // the grid size at the current hierarchy level
	float best_MCL = map_quads(nodes, level, level_step, dim, parent_node, is_mesh); // merge the nodes/clusters at this level
	cout << "Level: " << level << " --------- " << best_MCL << "---------" << endl;
	if(level == 1) { // base case, lowest-non trivial level
		if(dim == max_dim) { // if we are at full dimension, extract the best/first mapping from parent_node's children mapping and write it to a file
			map<long, point> &mapping = parent_node->children_mapping.all_rotations.begin()->mapping;			
			write_mapping(".htm", mapping, dims);
		}
		return best_MCL;
	}

	for (Node_rahtm* node : nodes) {
    	set<Node_rahtm*> children;
		node->get_set_children(children);
    	if (children.empty()) break; // if one node has no children at this level, then all other nodes have no children, so we can break here
    	best_MCL = recurse_quads2(level - 1, level_step, children, dim, node, is_mesh);
    	cout << "Level: " << level - level_step << " --------- " << best_MCL << "---------" << endl;
	}

	return best_MCL;
}

float map_graph()
{
	// -------------------- Phase 2: Top-Down Mapping of the Communication Graph Clusters --------------------
	point dims; // default constructor initializes a 2D point
	dims.set(2);
	int top_level_index = hier_nodes.size() - 1; // hier_nodes.size() = 3, so top_level_index = 2 (the coarsest graph)
	hier_mapping.resize(top_level_index + 1); // this stores the mapping solutions at different levels (needs enough space to store all the mappings at all levels)
	hier_mapping[top_level_index].setDims(dims); // at the highest level the mapping is done on a 2D point object (2x2 Grid)
	
	// ---------- Mapping Clusters at Top Level ----------
	if(IS_IMR)
		imr_map(hier_graphs[top_level_index]->adj, dims, hier_mapping[top_level_index]); // optimal MILP-based Mapping
	else
		HyperCubePartition::partitionHyperCube(hier_graphs[top_level_index]->adj, dims.size(), hier_mapping[top_level_index]); // heuristic-based mapping

	// If we have only one level, just write it to a file and evaluate its MCL and return
	if(top_level_index == 1) {
		dims.set(max_dim); // max_dim = 4
		write_mapping(".htm", hier_mapping[top_level_index].mapping, dims);
		return evaluate_mapping(hier_graphs[top_level_index]->adj, dims, hier_mapping[top_level_index].mapping, 0);
	}
	
	distribute_mapping(top_level_index, hier_mapping[top_level_index].mapping); // copies the solution "mapping" into the actual Node_rahtm objects at that hierarchy level
	
	// note1: At this point (in phase 2), the top level level is mapped entirely (no further processing is needed)

	// --------------------

	// ---------- Mapping Clusters at Subsequent (lower/finer) Level ----------
	// the top level has already been mapped above, so we start from the second level
	for(int level = top_level_index - 1; level >= 1; level--) { 
		for(auto nodes_it = hier_nodes[level+1].begin(); nodes_it != hier_nodes[level+1].end(); nodes_it++) {
			
			set<long> node_ids; nodes_it->second->get_set_children(node_ids); // those are the ids of the children nodes
			
			FlowSet fs; hier_graphs[level]->adj.getSubset(node_ids, fs);
			
			NodeMappings nm; nm.setDims(dims);
			
			if(nodes_it == hier_nodes[level+1].begin() || USE_ANALOGY_HYPERCUBES == false) { // if this is the first level, or analogy reuse is disabled
				if(IS_IMR == true)
					imr_map(fs, dims, nm);
				else
					HyperCubePartition::partitionHyperCube(fs, dims.size(), nm);
			} else {
				nm.fill_by_analogy(node_ids, hier_nodes[level+1].begin()->second->children_mapping.mapping);
			}
			
			nodes_it->second->children_mapping.fill(dims, nm); // save the resulting mapping to the children mapping of this node
		}
	}

	// -------------------- Phase 3: Bottom-Up Heurisitic Reorientation/Rotation and Merging --------------------
	// Insert all top-level nodes into a set. These will be the starting point of merging
	set<Node_rahtm*> nodes;
	for(auto nodes_it = hier_nodes[top_level_index].begin(); nodes_it != hier_nodes[top_level_index].end(); nodes_it++) 
		nodes.insert(nodes_it->second);
	
	float best_MCL;
	
	Node_rahtm root(0); // dummy node representing the root of the merging hierarhcy
	
	for(int level = top_level_index - 1, dim = 4; level >= 0; level--, dim *= 2) { // we start at top_level_index - 1, because: see note1
		cout << endl << "------------------------ " << dim << "x" << dim << " -----------------------" << endl;
		if (level > 0) best_MCL = recurse_quads2(level, top_level_index - level, nodes, dim, &root, level > 1); // this value is important only in the last level (level=1)
		// at the last iteration(level=1) best_MCL represents the MCL for the original communication graph, which is what we need to calculate
	}
	cout << endl << "------------------------   Final MCL = " << best_MCL << "  -----------------------" << endl;
	return best_MCL;
}
#endif

/*
	At top level (top_level_index), we have the 4 clusters stored into a 2x2 grid
	At next level (top_level_index - 1), we have the 16 clusters stored into a 4x4 grid (hence dim = 4)
*/
