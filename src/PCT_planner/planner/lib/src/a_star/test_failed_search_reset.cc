#include <Eigen/Core>

#include "a_star/a_star_search.h"

int main() {
  Eigen::MatrixXd cost_map = Eigen::MatrixXd::Zero(5, 5);
  const Eigen::MatrixXd empty_map = Eigen::MatrixXd::Zero(5, 5);

  // Isolate (4, 4) while leaving the rest of the map connected.  The first
  // search therefore expands the reachable component and fails after writing
  // g/f/parent state into its nodes.
  cost_map(3, 3) = 50.0;
  cost_map(3, 4) = 50.0;
  cost_map(4, 3) = 50.0;

  Astar astar;
  astar.Init(20.0, 1, 1.0, 0.5, cost_map, empty_map, empty_map,
             empty_map);

  const Eigen::Vector3i start(0, 0, 0);
  const Eigen::Vector3i unreachable_goal(0, 4, 4);
  const Eigen::Vector3i reachable_goal(0, 2, 0);

  if (astar.Search(start, unreachable_goal)) {
    return 1;
  }

  // This succeeds only if Search() clears state left by the failed request.
  if (!astar.Search(start, reachable_goal) || astar.GetPathPoints().empty()) {
    return 2;
  }

  return 0;
}
