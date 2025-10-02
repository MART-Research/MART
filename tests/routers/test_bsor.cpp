#include <cstdlib>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <utility>
#include <algorithm>
#include <cmath>
#include "../../modules/Router/BSOR.hpp"

int main(int argc, char* argv[]) {

    int OBJECTIVE_NUM = 3;
    std::multimap<int, std::pair<int, int>> flows {{1, {3, 10}} , {2, {3, 5}}};
    std::unordered_map<std::pair<int, int>, int, pair_hash> topo = { {{1, 2}, 20}, {{1, 3}, 9}, {{2, 3}, 15} };
    InitGoogle(argv[0], &argc, &argv, true);
    absl::SetFlag(&FLAGS_stderrthreshold, 0);
    BSOR bsor(3, topo, flows, OBJECTIVE_NUM, false, 0, 0);
    bsor.run_bsor();
    return EXIT_SUCCESS;
  }

  /*
  Testcases

    testcase 3:

      flows : {{1, {3, 10}} , {2, {3, 5}}}
      hop = {2, 2};
      topo = { {{1, 2}, 20}, {{1, 3}, 9}, {{2, 3}, 15} };
      OBJECTIVE_NUM = 3;
      
      expected objective value = 10 
      explanation of output:
      1st flow goes from 1 to 3 with a demand of 10 (note that the capacity of the edge from 1 to 3 is 9, but as we know the capacity constraints are not applicable in case of OBJECTIVE_NUM 3 (Minimization of MCL))
      2nd flow goes from 2 to 3 with a demand of 5
      hence the overall MCL is 10
      

      testcase 4:

      flows : {{1, {3, 10}} , {2, {3, 5}}}
      hop = {2, 1};
      topo = { {{1, 2}, 20}, {{1, 3}, 9}, {{2, 3}, 15} };
      OBJECTIVE_NUM = 1; // or 2 or 3
      
      expected objective value = NO solution
      explanation of output:
      the hop value for the 2nd flow is only 1, whereas the minimum possible hop value for the path between nodes 1 and 2 (on the CDG) is 2, hence there is no solution

  */
