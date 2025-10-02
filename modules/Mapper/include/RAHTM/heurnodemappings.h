#ifndef HMAPPING_H
#define HMAPPING_H

#include <stdlib.h>
#include <vector>
#include <map>
#include <set>
#include <ostream>
#include <algorithm>
#include <list>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "point.h"
#include "nodemappings.h"

using namespace std;

#define N_FORKS 256

/*
	In RAHTM’s Phase 3 (“bottom-up heuristic merging”), each leaf block (a small 2-ary n-cube that was optimally mapped in Phase 2) 
	can be re-oriented many different ways:
		Rotate axes (e.g., swap X↔Y, cycle X→Y→Z, …).
 		Flip along an axis (mirror).
	
	Trying all orientations of all blocks explodes combinatorially.
	HeurNodeMappings is the wrapper that:
		1. pre-computes every orientation of one block and stores them in all_rotations.
		2. Keeps track of a current choice (rotation_id) for stochastic search.
		3. Provides helpers to move the block (offset / mod-wrap) when it is fused into a larger cube.
		4. Supplies routines that pairwise-merge two blocks while pruning the search space.

	So, conceptually, NodeMappings = one mapping,
	HeurNodeMappings = set of all rotated/translated versions of that mapping plus bookkeeping.


*/


class HeurNodeMappings: public NodeMappings {
public:
	
	vector<NodeMappings> all_rotations; // Cache of every rotation/flip variant of one sub-cube mapping
	long rotation_id; // index of the currently selected rotation within all_rotations

	HeurNodeMappings() {
		rotation_id = 0;
	}

	void incRotationId() {
		rotation_id = (rotation_id + 1) % all_rotations.size(); // cycle to the next orientation
	}
	void fill(point &dims, NodeMappings nm) {
		setDims(dims);
		mapping = nm.mapping;

		all_rotations.clear();
		all_rotations.reserve(nm.max_rotations);
		
		do {
			all_rotations.push_back(nm);
			nm.rotateMapping();
		} while(nm.rotation_seq < nm.max_rotations);
	}
	void adjustMappings(point &st) {
		for (auto& rotation : all_rotations) {
			rotation.adjustMappings(st);
		}
		NodeMappings::adjustMappings(st);
	}

	void adjustMappings(point &st, point &dims) {
		for (auto& rotation : all_rotations) {
			rotation.adjustMappings(st, dims);
		}
		NodeMappings::adjustMappings(st);
	}

	inline void reapAllChildren(
		std::map<pid_t, std::pair<std::vector<NodeMappings>::iterator, std::vector<NodeMappings>::iterator>>& fork_pids,
		std::map<pid_t, int>& child_pipe)
	{
		int status;
		while (!fork_pids.empty()) 
		{
			auto it = fork_pids.begin();
			pid_t PID = it->first;
			char readbuffer[32];
			ssize_t temp = read(child_pipe[PID], readbuffer, sizeof(readbuffer));
			close(child_pipe[PID]);
			waitpid(PID, &status, 0);
			float MCL = std::atof(readbuffer);
			it->second.first->score += MCL;
			it->second.second->score += MCL;
			fork_pids.erase(it);
			child_pipe.erase(PID);
		}
	}
	
	void heurRotateAndMergeMapping(HeurNodeMappings &nm2, FlowSet &fs, int dim, bool is_mesh) {
		
		NodeMappings nm;
		
		point dims_merged(dims);
		dims_merged[dim] = this->dims[dim] + nm2.dims[dim];
		nm.setDims(dims_merged);

		// Reset scores
		vector<NodeMappings>::iterator it1 = all_rotations.begin();
		/*for(; it1 != all_rotations.end(); it1++)
			it1->resetScore();*/

		vector<NodeMappings>::iterator it2 = nm2.all_rotations.begin();
		/*for(; it2 != nm2.all_rotations.end(); it2++)
			it2->resetScore();*/
		
		map<pid_t, pair<vector<NodeMappings>::iterator, vector<NodeMappings>::iterator> > fork_pids;
		map<pid_t, int> child_pipe;
		// Try all combinations of rotations between the two k-ary n-cubes
		for(it1 = all_rotations.begin(); it1 != all_rotations.end(); it1++) {

			it2 = nm2.all_rotations.begin();
			for(it2 = nm2.all_rotations.begin(); it2 != nm2.all_rotations.end(); it2++) {

				nm.addMappings(*it1);
				nm.addMappings(*it2);
				// nm now cholds a concrete merged layout of the two mappings

				// calls to evaluate_mapping are expensive, so we parallelize them
				if(fork_pids.size() >= N_FORKS) // saturation: reap children (collect the results)
				{
					reapAllChildren(fork_pids, child_pipe);
				} 
				else 
				{
					int fd[2];
					int temp = pipe(fd);
					pid_t PID = fork();
					if(PID == -1)
					{
						perror("fork");
						exit(1);
					} 
					else if(PID == 0) // child branch
					{
						close(fd[0]);
						float MCL = evaluate_mapping(fs, dims_merged, nm.mapping, is_mesh);
						stringstream ss; ss << MCL;
						ssize_t temp = write(fd[1], ss.str().c_str(), ss.str().size()+1);
						exit(0);
					} 
					else 
					{
						close(fd[1]);
						child_pipe[PID] = fd[0];
						fork_pids[PID] = pair<vector<NodeMappings>::iterator, vector<NodeMappings>::iterator>(it1, it2);
					}
				}
				//it1->score = MCL < it1->score ? MCL : it1->score;
				//it2->score = MCL < it2->score ? MCL : it2->score;
			}
		}
		
		reapAllChildren(fork_pids, child_pipe); // reap all remaining children if any

		// Sort each list of rotations ascendingly according to best score possible
		sort(all_rotations.begin(), all_rotations.end(), nm2);
		sort(nm2.all_rotations.begin(), nm2.all_rotations.end(), nm2);

		// Cut half the list, generally the list has size 2^n*nDims, after cutting the list into half (n times), nDims will remain
		//all_rotations.resize(all_rotations.size() >> 1);
		//nm2.all_rotations.resize(nm2.all_rotations.size() >> 1);
	}

	/*
	Tests rotation combinations
	Optimization: Prunes poor candidates early
	*/
	static float incrementalRotations(vector<NodeMappings> &all_rotations1, 
		vector<NodeMappings> &all_rotations2, vector<NodeMappings> &all_rotations_merged,
		FlowSet &fs, long size_limit, point &dims, bool is_mesh) 
		{
		
		all_rotations_merged.clear();
		all_rotations_merged.reserve(size_limit);

		multimap<float, NodeMappings> mcl_rot; // a mapping container for the best MCL scores (MCL, mapping), sorted in ascending order (best MCL first)

		//dims.include(all_rotations2.begin()->mapping);

		NodeMappings nm; nm.setDims(dims);

		// Reset scores
		vector<NodeMappings>::iterator it1 = all_rotations1.begin();

		map<pid_t, NodeMappings> fork_pids;
		map<pid_t, int> child_pipe;
		
		// Try all combinations of rotations between the two k-ary n-cubes
		// The nested loop enumerates all combinations of rotations between the two k-ary n-cubes
		for(it1 = all_rotations1.begin(); it1 != all_rotations1.end(); it1++) 
		{
			for(auto it2 = all_rotations2.begin(); it2 != all_rotations2.end(); it2++) 
			{

				// nm.mapping.clear();
				nm.addMappings(*it1);
				nm.addMappings(*it2);

				/**
				 * Since evaluate_mapping is time consuming, we parallelize the calls to it (up to N_FORKS processes in parallel)
				 * If we already have N_FORKS subprocesses running, we stop spawning and harvest finished children first.
				 * After that, we can spawn new processes again.
				 * 
				*/
				if(fork_pids.size() >= N_FORKS) // collect child processes results
				{
					int status;
					while(fork_pids.size() > 0) // Loop until all outstanding children have been collected.
					{
						auto it = fork_pids.begin();
						pid_t PID = it->first;
						char readbuffer[32];
						ssize_t temp = read(child_pipe[PID], readbuffer, sizeof(readbuffer));
						close(child_pipe[PID]);
						waitpid(PID, &status, 0); // Remove the finished child from the process table (in other words: reap the zombie child process).
						float MCL = atof(readbuffer);
						//cout << "Rec: " << readbuffer << " " << MCL << endl;
						mcl_rot.insert(pair<long, NodeMappings> (MCL, it->second)); // insert the returned MCL and the corresponding mapping into the multimap
						
						// To cap the search space, we retain only the size_limit best candidates, deleting the worst (largest MCL) entries from the end of the multimap.
						while(mcl_rot.size() > size_limit) { 
							multimap<float, NodeMappings>::iterator mit = mcl_rot.end();
							mit--;
							mcl_rot.erase(mit);
						}
					
						fork_pids.erase(it);
						child_pipe.erase(PID);
					}
				} 
				else // 
				{
					// create a pipe for parent-child communication
					int fd[2];
					int temp = pipe(fd);
					pid_t PID = fork();
					if(PID == -1)
					{
						perror("fork");
						exit(1);
					} 
					else if(PID == 0) // child process
					{
						close(fd[0]); // close the read end of the pipe (the child process will only write to the pipe)
						float MCL = evaluate_mapping(fs, dims, nm.mapping, is_mesh);
						stringstream ss; ss << MCL;
						ssize_t temp = write(fd[1], ss.str().c_str(), ss.str().size()+1);
						exit(0);
					} 
					else // parent process
					{
						close(fd[1]); // close the write end of the pipe (the parent process will only read from the pipe)
						child_pipe[PID] = fd[0]; // Stores the read-FD in child_pipe so we can later read() the result when the child finishes.						
						fork_pids[PID] = nm;; // remember which mapping this PID owns
					}
				}
			}
		}
		
		int status;
		while(fork_pids.size() > 0) 
		{
			auto it = fork_pids.begin();
			pid_t PID = it->first;
			
			char readbuffer[32];
			ssize_t temp = read(child_pipe[PID], readbuffer, sizeof(readbuffer));
			close(child_pipe[PID]);
			waitpid(PID, &status, 0);
			float MCL = atof(readbuffer);
			//cout << "Rec: " << readbuffer << " " << MCL << endl;
			mcl_rot.insert(pair<long, NodeMappings> (MCL, it->second));
			while(mcl_rot.size() > size_limit) {
				multimap<float, NodeMappings>::iterator mit = mcl_rot.end();
				mit--;
				mcl_rot.erase(mit);
			}
		
			fork_pids.erase(it);
			child_pipe.erase(PID);
		}

		for(auto mit = mcl_rot.begin(); mit != mcl_rot.end(); mit++)
			all_rotations_merged.push_back(mit->second);

		cout << "Incremental return " << mcl_rot.begin()->first << endl;
		return mcl_rot.begin()->first;
	}
	
	// A fallback stochastic search (random walk on rotation indices) if the exhaustive pairwise merge is disabled. 
	// Not used in default build.
	 
	static void iterativeMergingMapping(vector<HeurNodeMappings*> heurs, FlowSet &fs, int dim, bool is_mesh) {
		NodeMappings nm;
		
		point dims_merged;
		dims_merged.set(dim);

		vector<long> rot_ids(heurs.size(), 0), best_rot(heurs.size(), 0);
		float MCL, best_MCL;

		collectMappings(heurs, nm);
		best_MCL = MCL = evaluate_mapping(fs, dims_merged, nm.mapping, is_mesh);

		for(int iter = 0; iter < 500; iter++) 
		{
			int i = rand() % (rot_ids.size() / 2);
			
			heurs[i]->incRotationId();
			rot_ids[i] = heurs[i]->rotation_id;

			collectMappings(heurs, nm);
			MCL = evaluate_mapping(fs, dims_merged, nm.mapping, is_mesh);

			if(MCL < best_MCL) 
			{
				best_MCL = MCL;
				best_rot = rot_ids;
			}
		}
	}

	static void collectMappings(vector<HeurNodeMappings*> heurs, NodeMappings &nm)
	{
		for(auto it = heurs.begin(); it != heurs.end(); it++) {
			nm.addMappings((*it)->all_rotations[(*it)->rotation_id]);
		}
	}
};
#endif
