#include "../../modules/Mapper/include/RAHTM/RAHTM.hpp"

int main() {

    int sz = 4096;
    int n_levels = 2;
    std::string benchmark = "cg.64"; // placed in same directory as the executable
    std::vector<point> partition_window(4, 0);
    partition_window[0] = 4;
    partition_window[1] = 4;
    partition_window[2] = 2;
    partition_window[3] = 2;

    bool skip_cols = false;
    float wt_thresh = 1e5;
    int sz_sqrt = sqrt(sz);
    point graph_dims(2);
    graph_dims.set(sz_sqrt);
    int conc_factor = 1;
    read_flows(bench_name.c_str(), conc_factor, skip_cols, wt_thresh);

    // Hierarchical Graph Clustering
    for(int i = no_conc; i < n_levels; i++) {
        float intra_cluster_sum, inter_cluster_sum;

        Graph *graph_up = hier_graphs[i]->collapse_graph(dims, intra_cluster_sum, inter_cluster_sum, partition_window[i], graph_dims); // new graph at next/upper level in the hierarchy

        hier_graphs.push_back(graph_up);

        // cout << "At level " << i + 1 << ": intra-cluster sum " << intra_cluster_sum << ", inter-cluster sum " << inter_cluster_sum << endl;

        dims.div(2);

        graph_dims[0] = graph_dims[0] / partition_window[i][0];
        graph_dims[1] = graph_dims[1] / partition_window[i][1];
    }

    float HTM_MCL = map_graph(); // RAHTM mapping

    std::cout << "HTM_MCL: " << HTM_MCL << std::endl;


}