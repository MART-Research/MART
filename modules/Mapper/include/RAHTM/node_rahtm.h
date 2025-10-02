/*
The Node_rahtm class is designed to represent a hierarchical structure where each node can have a parent and multiple children.
	The functions provide mechanisms to:
	Set up parent-child relationships.
	Retrieve children or their mappings at various levels of the hierarchy.
	Organize nodes globally using the hier_nodes structure for efficient access.
*/
#ifndef NODE_H
#define NODE_H
#include <vector>
#include <map>
#include <set>
#include "point.h"
#include "heurnodemappings.h"

using namespace std;

class Node_rahtm {
public:
	Node_rahtm *parent;
	map<long, Node_rahtm*> children;
	
	long id;
	int level;
	point mapping;
	HeurNodeMappings children_mapping;

	Node_rahtm(long id) { init(id, 0); }
	Node_rahtm(long id, int level) { init(id, level); }
	void init(long id, int level) {
		this->id = id;
		this->level = level;
		parent = NULL;
	}
	void set_parent(Node_rahtm *parent) { this->parent = parent; }
	void add_child(Node_rahtm *child) { children[child->id] = child; }

	// ---- Child retrieval functions ----

	// populates the children set with all children of this node
	void get_set_children(set<Node_rahtm*> &children_set) {
		for(auto child_it = children.begin(); child_it != children.end(); child_it++)
			children_set.insert(child_it->second);
	}

	// populates the children set with the IDs of all children of this node
	void get_set_children(set<long> &children_set) {
		for(auto child_it = children.begin(); child_it != children.end(); child_it++)
			children_set.insert(child_it->second->id);
	}

	// populates the children set with all children of this node at a specific level
	void get_n_level_set_children(int level_step, set<Node_rahtm*> &children_set) {
		if(children.size() == 0) return;
		
		if(level_step == 1) { // base case (level 1)
			get_set_children(children_set);
		} else if(level_step > 1) {
			for(auto child_it = children.begin(); child_it != children.end(); child_it++)
				child_it->second->get_n_level_set_children(level_step - 1, children_set);
		}
	}
	// --------------------------------------

	// ---- Mapping retrieval functions ----

	// populates the children set with the IDs and mapping of all child nodes of the current node
	// void get_children_mapping(map<long, point > &children_mapping) {
	// 	for(auto child_it = children.begin(); child_it != children.end(); child_it++)
	// 		children_mapping[child_it->first] = child_it->second->mapping;
	// }

	// // populates the children set with the IDs and mapping of all child nodes of the current node at a specific level
	// void get_n_level_children_mapping(int level_step, map<long, point > &children_mapping) {
	// 	if(children.size() == 0)
	// 		return;

	// 	if(level_step == 1) { // base case (level 1)
	// 		get_children_mapping(children_mapping);
	// 	} else if(level_step > 1) {
	// 		for(auto child_it = children.begin(); child_it != children.end(); child_it++)
	// 			child_it->second->get_n_level_children_mapping(level_step - 1, children_mapping);
	// 	}
	// }
	// --------------------------------------
};
#endif