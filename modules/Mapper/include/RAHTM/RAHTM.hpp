#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <math.h>
#include "point.h"
#include "flowset.h"
#include "graph.h"
#include "hypercubepartition.h"
#include "map_graph.h"

#include "hilbert_c.h"

using namespace std;

point dims;
vector<Graph*> hier_graphs;
vector<map<long, Node_rahtm*> > hier_nodes;
vector<NodeMappings> hier_mapping;

string bench_name;
int conc_factor; // concentration factor
bool no_conc;

int max_dim;

void get_neighbors(vector<int> &node, vector<vector<int> > &neighbors) {
	neighbors.clear();
	for(int d = 0; d < dims.size(); d++) {
		if(node[d] > 0) {
			neighbors.push_back(node);
			neighbors[neighbors.size()-1][d]--;
		} 
		if(node[d] < dims[d]-1) {
			neighbors.push_back(node);
			neighbors[neighbors.size()-1][d]++;
		}
	}
}

void conv_to_dims(long node, vector<int> &node_dims)
{
	node_dims.clear();
	node_dims.resize(dims.size(),0);
	int d = dims.size()-1;
	while(node > 0) {
		node_dims[d] = node % dims[d];
		node = node / dims[d];
		d--;
	}
}

long conv_from_dims(vector<int> &node_dims)
{
	long node = 0;
	
	for(int d = 0; d < dims.size(); d++) {
		node = node * dims[d] + node_dims[d];
	}
	return node;
}

void dump_node_dims(FILE *fp, string prefix, vector<int> node_dims)
{
	assert(node_dims.size() == dims.size());
	fprintf(fp, "%s", prefix.c_str());
	for(int d = 0; d < dims.size(); d++) {
		fprintf(fp, "_%d", node_dims[d]);
	}
}

/*
	file_name: the file to read flows from (like bench.flows).
	conc: concentration factor — used to group multiple threads/processes into one node.
	skip_cols: whether to skip two fields in the input file (sometimes extra columns are present).
	wt_thresh: weight threshold scaling — divides the raw weight.
*/

void read_flows(string file_name, int conc, bool skip_cols, float wt_thresh) {
	hier_graphs.push_back(new Graph(hier_graphs.size())); // the original communication graph
	hier_nodes.resize(hier_nodes.size()+1); // size = 1
	int index = hier_graphs.size() - 1; // 0
	
	ifstream fp_in(file_name.c_str(), ifstream::in);
	if(!fp_in.is_open()) {
		cout << "Error reading file " << file_name << endl << endl;
		fp_in.close();
		exit(1);
	}
	string line;
	while(getline(fp_in,line)) {
		stringstream ss; ss.str(line);
		long src, dest; float temp1, temp2, wt;
		ss >> src;
		if(skip_cols) ss >> temp1 >> temp2;
		ss >> dest >> wt;
		src = src / conc; dest = dest / conc;
		wt = wt / wt_thresh;
		if(src != dest && wt > 1) {
			
			if(hier_nodes[index].find(src) == hier_nodes[index].end())
				hier_nodes[index][src] = new Node_rahtm(src);
			
			if(hier_nodes[index].find(dest) == hier_nodes[index].end())
				hier_nodes[index][dest] = new Node_rahtm(dest);
			
			hier_graphs[index]->add_edge(src, dest, wt);
		}
	}
}


float direct_mapping(point &dims)
{
	point p;
	map<long, point> mapping;
	long i = 0;

	do {
		mapping[i] = p;
		i++;
	} while(p.inc(dims));

	float direct_MCL = evaluate_mapping(hier_graphs[1]->adj, dims, mapping, 0);

	write_mapping(".row", mapping, dims);
	
	if(dims.n == 3) {
		for(auto it = mapping.begin(); it != mapping.end(); it++) {
			if(it->second[2] & 1) { // odd, flip dimension
				it->second[1] = dims[1] - it->second[1] - 1;
				it->second[0] = dims[0] - it->second[0] - 1;
			} //else	if(it->second[1] & 1) { // odd, flip dimension
				//it->second[0] = dims[0] - it->second[0] - 1;
			//}
		}
		float Z_MCL = evaluate_mapping(hier_graphs[1]->adj, dims, mapping, 0);
		write_mapping(".alt", mapping, dims);
		cout << "Snake MCL " << Z_MCL << endl;
	}
	
	return direct_MCL;
}