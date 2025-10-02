#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <iostream>
#include <iomanip>
#include "node_rahtm.h"
#include "flowset.h"
using namespace std;

extern vector<map<long, Node_rahtm*> > hier_nodes; 
// vector of maps to store nodes at each hierarchy level, each vector index corresponds to a level, each map stores nodes at that level


extern "C" {int       interface(int       nvtxs,		/* number of vertices in full graph */
						int      *start,		/* start of edge list for each vertex */
						int      *adjacency,		/* edge list data */
						int      *vwgts,		/* weights for all vertices */
						float    *ewgts,		/* weights for all edges */
						float    *x, float *y, float *z,		/* coordinates for inertial method */
						char     *outassignname,	/* name of assignment output file */
						char     *outfilename,		/* output file name */
						short    *assignment,		/* set number of each vtx (length n) */
						int       architecture,		/* 0 => hypercube, d => d-dimensional mesh */
						int       ndims_tot,		/* total number of cube dimensions to divide */
						int       mesh_dims[3],		/* dimensions of mesh of processors */
						double   *goal,			/* desired set sizes for each set */
						int       global_method,	/* global partitioning algorithm */
						int       local_method,		/* local partitioning algorithm */
						int       rqi_flag,		/* should I use RQI/Symmlq eigensolver? */
						int       vmax,			/* how many vertices to coarsen down to? */
						int       ndims,		/* number of eigenvectors (2^d sets) */
						double    eigtol,		/* tolerance on eigenvectors */
						long      seed			/* for random graph mutations */
						);}
// Assumes vertices are numbered 0 to n-1
class Graph {
	public:	
		FlowSet adj;
		vector<long>mapping;
		int level;
		bool is_mapped, is_assigned;
		map<long, long>node_partition_assign; // <key: node, value: partition_id> mapping of nodes to partitions

		// n vertices and m edges
		Graph(int level) { this->level = level; }		
		Graph() { level = 0; }
		void add_edge(long a, long b, float wt) { adj.addFlow(a, b, wt); }
		~Graph() {}
		long n_vertices() { return adj.nodes.size();} // number of vertices in the communication graph
		long n_edges() { return adj.n_flows; }
		
		// collapse graph into a coarser grained graph level
		// dims: dimension of target partition
		// intra_cluster_sum: sum of intra-partition weights
		// inter_cluster_sum: sum of inter-partition weights
		// graph_window: size of each partition window
		// graph_dims: original graph dimensions
		Graph* collapse_graph(point dims, float &intra_cluster_sum, float &inter_cluster_sum, point &graph_window, point &graph_dims) {
			Graph* new_graph;
			long vertices_node = ceil(((float)n_vertices()) / dims.get_prod()); // expected number of vertices per cluster/partition, dims.get_prod() = product of partition dimensions = no. of compute nodes in the partition/topology
			if(n_vertices() % dims.get_prod()) { //dims_tot) {
				cout << "Graph is not evenly partitioned: " <<  n_vertices() << " " << dims.get_prod() << endl; // << dims_tot;
			}
			point new_dims(graph_window.n); // stores dimensions of coarsed graph partitions
			new_dims[0] = graph_dims[0] / graph_window[0];
			new_dims[1] = graph_dims[1] / graph_window[1];
			// ex: original [8, 8] graph, window [2, 2] -> new graph [4, 4]
			
			// Node_rahtm to partition mapping (This is supposed to be the smart clustering/partitioning, and trial of different tiles, but actually it is a simple approach)
			for(long v = 0; v < n_vertices(); v++) {
				int v_col = v % graph_dims[0]; // col of vertex v in current graph (value in range [0, graph_dims[0]-1])
				int v_row = v / graph_dims[0]; // row of vertex v in current graph (value in range [0, graph_dims[1]-1])
				
				int new_col = v_col / graph_window[0]; // col of vertex v in new graph partition
				int new_row = v_row / graph_window[1]; // row of vertex v in new graph partition
				
				node_partition_assign[v] = new_row*new_dims[0] + new_col; // linearize partition coordinates into a node id
			}
			
			// Print the assignment of each node (which partition it belongs to)
			/*
			for(long i = 0 ; i < n_vertices(); i++) {
				if(i % 16 == 0)
					cout << endl;
				cout << setw(2) << node_partition_assign[i] << " ";
			}
			cout << endl;
			*/
			
			intra_cluster_sum = inter_cluster_sum = 0;
			long level_up = level + 1; // next hierarchy level
			hier_nodes.resize(level_up + 1); // ensure storage for parent nodes at next level
			cout << "Current Size of hier_nodes: " << hier_nodes.size() << endl;

			new_graph = new Graph(level + 1); // create new graph for next level
			
			for(auto flow_it = adj.flows.begin(); flow_it != adj.flows.end(); flow_it++) { // iterates through all flows in the graph
				long flowSrc = flow_it->first; // source node id
				for(auto it = flow_it->second.begin(); it != flow_it->second.end(); it++) { // iterates through all the destination nodes of the flow
					long flowDst = it->first; // destination node id
					
					// ------ Handle flowSrc ---------
					long g1 = node_partition_assign[flowSrc]; // id of partition to which source node flowSrc was assigned			
					Node_rahtm* node1; // the node at the next level (level_up) to which the current node (flowSrc) will be assigned
					
					// check if node already exists. if not, create it, if yes, get it from hier_nodes
					if( hier_nodes[level_up].find(g1) == hier_nodes[level_up].end()) 
					{
						node1 = new Node_rahtm(g1, level_up); // the constructor sets the node id and level, and adds/saves the node to the hier_nodes -> no need to explicitly add it here
						hier_nodes[level_up][g1] = node1;
					}
					else 
					{
						node1 = hier_nodes[level_up].find(g1)->second; // retrieve the node from the map
					}
						
					// add the current node to the children of the created node (which is one level up), and make the created node the parent of the current node
					node1->add_child(hier_nodes[level].find(flowSrc)->second);
					hier_nodes[level][flowSrc]->set_parent(node1);

					

					long g2 = node_partition_assign[flowDst]; // id of partition to which destination node flowDst was assigned
					Node_rahtm* node2; // the node at the next level (level_up) to which the current node (flowDst) will be assigned
					if( hier_nodes[level_up].find(g2) == hier_nodes[level_up].end()) 
					{
						node2 = new Node_rahtm(g2, level_up);
						hier_nodes[level_up][g2] = node2;
					} 
					else {
						node2 = hier_nodes[level_up].find(g2)->second;
					}

					node2->add_child(hier_nodes[level].find(flowDst)->second);
					hier_nodes[level][flowDst]->set_parent(node2);

					// if the source and destination nodes are in the same partition, their communication load is intra-partition
					if(g1 == g2) {
						intra_cluster_sum += it->second;
					} else { // else, the communication load is inter-partition, and we need to add an inter-partition edge between the two partitions (over which the src and destination nodes of the flow will communicate across partitions)
						new_graph->add_edge(g1, g2, it->second); // this actually adds a flow to the FlowSet adj
						inter_cluster_sum += it->second;
					}
				}
			}
			return new_graph;
		}
		
		void output_graph(ostream &o) {
			for(auto flow_it = adj.flows.begin(); flow_it != adj.flows.end(); flow_it++) {
				for(map<long, float>::iterator it = flow_it->second.begin(); it != flow_it->second.end(); it++) {
					if(it != flow_it->second.begin()) o << " ";
					o << it->first << " " << it->second;
				}
				if(flow_it->second.size() > 0)
					o << endl;
			}
		}
		void output_graph() { output_graph(cout); }

	private:
};
#endif