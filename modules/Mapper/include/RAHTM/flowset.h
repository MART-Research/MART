#ifndef FLOWSET_H
#define FLOWSET_H

#include <map>
#include <set>
#include <iostream>

using namespace std;

class FlowSet
{
public:
    
    map<long, map<long, float> > flows;         // Nested map: source -> (destination -> weight)
    set<long> nodes;                             // Set of all unique nodes in the flow set
    float total_wt;                              // Sum of all flow weights
    float max_wt;                                // Maximum weight among all flows
    long n_flows;                                // Count of unique flows

    
    FlowSet() { reset(); }
    
    // Resets all flow data to initial empty state
    void reset() {
        total_wt = max_wt = 0;
        n_flows = 0;
        flows.clear();
        nodes.clear();
    }

    // Adds/updates a flow between source and destination nodes with given weight.
    // Updates total weight, max weight, and flow count accordingly.
    inline void addFlow(long src, long dest, float wt) {
        auto& dest_map = flows[src];  // Get or create the inner map
        auto it = dest_map.find(dest); // we cannot use [] here, because it will try create a new entry if it doesn't exist, which we don't want, at least not yet
        if (it != dest_map.end()) {
            // Flow exists(a flow from the same souce node to the same destination node exists, so we update it): so update weight and stats
            it->second += wt;
            max_wt = std::max(max_wt, it->second);
        } else {
            // New flow: insert and update stats
            dest_map.emplace(dest, wt);
            max_wt = std::max(max_wt, wt);
            n_flows++;
            nodes.insert(src); nodes.insert(dest);
        }
        total_wt += wt;
    }

    // Splits flows into two subsets based on node membership:
    // - fs1: flows where both endpoints are in 'nodes' set
    // - fs2: flows where both endpoints are NOT in 'nodes' set
    // Flows with mixed membership (one in, one out) are discarded
    void splitFlows(const set<long>& nodes, FlowSet& fs1, FlowSet& fs2) {
        for (const auto& [src, destinations] : flows) {
            bool src_in_nodes = nodes.count(src) > 0;
            
            for (const auto& [dest, weight] : destinations) {
                bool dest_in_nodes = nodes.count(dest) > 0;
                
                if (src_in_nodes && dest_in_nodes) {
                    fs1.addFlow(src, dest, weight);
                } else if (!src_in_nodes && !dest_in_nodes) {
                    fs2.addFlow(src, dest, weight);
                }
                // Mixed case: implicitly discarded
            }
        }
    }
    
    // Creates a subset of flows where both endpoints are in the specified node set
    void getSubset(const set<long>& nodes, FlowSet& fs) {
        for (const auto& [src, destinations] : flows) {
            if (!nodes.count(src)) continue; // Skip if source node is not in the set

            for (const auto& [dest, weight] : destinations) {
                if (nodes.count(dest)) { // Add flow only if both src and dest are included
                    fs.addFlow(src, dest, weight);
                }
            }
        }

        fs.nodes = nodes; // Assign the node list for the subset
    }
};

#endif