#include "art_planner/planners/grid_astar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include <grid_map_core/iterators/LineIterator.hpp>

#include "art_planner/utils.h"



using namespace art_planner;



namespace {

struct Cell {
  int x{0};
  int y{0};
};

struct QueueNode {
  int index{0};
  double f{0.0};

  bool operator<(const QueueNode& other) const {
    return f > other.f;
  }
};

struct SearchMap {
  grid_map::GridMap map;
  std::string elevation_layer;
  std::string traversability_layer;
  bool has_traversability_thresholded{false};
  bool has_observed{false};
  int size_x{0};
  int size_y{0};
  double resolution{0.0};
};

constexpr double kUnknownObservedThreshold = 0.5;
constexpr double kTraversabilityThresholded = 0.5;

int toFlatIndex(const Cell& cell, int size_y) {
  return cell.x * size_y + cell.y;
}

Cell fromFlatIndex(int index, int size_y) {
  return Cell{index / size_y, index % size_y};
}

bool isInside(const Cell& cell, const SearchMap& search_map) {
  return cell.x >= 0 && cell.y >= 0 &&
         cell.x < search_map.size_x && cell.y < search_map.size_y;
}

grid_map::Index toGridIndex(const Cell& cell) {
  return grid_map::Index(cell.x, cell.y);
}

grid_map::Position cellPosition(const SearchMap& search_map, const Cell& cell) {
  grid_map::Position position;
  search_map.map.getPosition(toGridIndex(cell), position);
  return position;
}

double distance2d(const grid_map::Position& a, const grid_map::Position& b) {
  return std::hypot(a.x() - b.x(), a.y() - b.y());
}

double pointLineDistance(const grid_map::Position& point,
                         const grid_map::Position& start,
                         const grid_map::Position& goal) {
  const double dx = goal.x() - start.x();
  const double dy = goal.y() - start.y();
  const double length = std::hypot(dx, dy);
  if (length <= std::numeric_limits<double>::epsilon()) {
    return distance2d(point, start);
  }
  return std::abs(dy * point.x() - dx * point.y() +
                  goal.x() * start.y() - goal.y() * start.x()) / length;
}

double directionAngle(int direction) {
  static const double angles[8] = {
      0.0,
      M_PI_4,
      M_PI_2,
      3.0 * M_PI_4,
      M_PI,
      -3.0 * M_PI_4,
      -M_PI_2,
      -M_PI_4,
  };
  return angles[direction];
}

double angleDiff(double a, double b) {
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

float layerValue(const SearchMap& search_map,
                 const std::string& layer,
                 const Cell& cell) {
  return search_map.map.at(layer, toGridIndex(cell));
}

bool isUnknownCell(const SearchMap& search_map, const Cell& cell) {
  if (search_map.has_observed) {
    const float observed = layerValue(search_map, "observed", cell);
    return !std::isfinite(observed) || observed <= kUnknownObservedThreshold;
  }
  const float elevation = layerValue(search_map, search_map.elevation_layer, cell);
  return !std::isfinite(elevation);
}

double traversabilityValue(const SearchMap& search_map, const Cell& cell) {
  const std::string layer = search_map.has_traversability_thresholded
      ? "traversability_thresholded"
      : search_map.traversability_layer;
  const float traversability = layerValue(search_map, layer, cell);
  if (!std::isfinite(traversability)) {
    return 0.0;
  }
  return std::max(0.0, std::min(1.0, static_cast<double>(traversability)));
}

bool isTraversableCell(const SearchMap& search_map,
                       const ParamsConstPtr& params,
                       const Cell& cell,
                       const grid_map::Position& start_position) {
  if (!isInside(cell, search_map)) {
    return false;
  }
  const auto position = cellPosition(search_map, cell);
  const bool unknown = isUnknownCell(search_map, cell);
  if (unknown &&
      distance2d(position, start_position) <=
          params->planner.astar.allow_unknown_start_radius) {
    return true;
  }
  if (unknown && params->planner.unknown_space_untraversable) {
    return false;
  }
  if (search_map.has_traversability_thresholded) {
    return traversabilityValue(search_map, cell) > kTraversabilityThresholded;
  }
  return traversabilityValue(search_map, cell) > params->planner.traversability_thres;
}

bool nearestValidCell(const SearchMap& search_map,
                      const ParamsConstPtr& params,
                      const Cell& seed,
                      const grid_map::Position& start_position,
                      double radius,
                      Cell* result) {
  if (isTraversableCell(search_map, params, seed, start_position)) {
    *result = seed;
    return true;
  }

  const int radius_cells = std::max(1, static_cast<int>(
      std::ceil(radius / std::max(search_map.resolution, 1e-6))));
  const auto seed_position = cellPosition(search_map, seed);
  double best_distance = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      const Cell candidate{seed.x + dx, seed.y + dy};
      if (!isTraversableCell(search_map, params, candidate, start_position)) {
        continue;
      }
      const auto candidate_position = cellPosition(search_map, candidate);
      const double candidate_distance = distance2d(seed_position, candidate_position);
      if (candidate_distance <= radius && candidate_distance < best_distance) {
        best_distance = candidate_distance;
        *result = candidate;
        found = true;
      }
    }
  }
  return found;
}

bool lineIsValid(const SearchMap& search_map,
                 const ParamsConstPtr& params,
                 const Cell& start,
                 const Cell& end,
                 const grid_map::Position& start_position) {
  const auto start_pos = cellPosition(search_map, start);
  const auto end_pos = cellPosition(search_map, end);
  grid_map::LineIterator iterator(search_map.map,
                                  start_pos,
                                  end_pos);
  for (; !iterator.isPastEnd(); ++iterator) {
    const auto index = *iterator;
    const Cell cell{index.x(), index.y()};
    if (!isTraversableCell(search_map, params, cell, start_position)) {
      return false;
    }
  }

  const double step = params->planner.astar.shortcut_max_step;
  if (step > std::numeric_limits<double>::epsilon()) {
    const double length = distance2d(start_pos, end_pos);
    const int n_steps = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int i = 1; i < n_steps; ++i) {
      const double ratio = static_cast<double>(i) / n_steps;
      const grid_map::Position sample(start_pos.x() + ratio * (end_pos.x() - start_pos.x()),
                                      start_pos.y() + ratio * (end_pos.y() - start_pos.y()));
      grid_map::Index sample_index;
      if (!search_map.map.getIndex(sample, sample_index)) {
        return false;
      }
      const Cell sample_cell{sample_index.x(), sample_index.y()};
      if (!isTraversableCell(search_map, params, sample_cell, start_position)) {
        return false;
      }
    }
  }
  return true;
}

std::vector<Cell> shortcutPath(const SearchMap& search_map,
                               const ParamsConstPtr& params,
                               const std::vector<Cell>& path,
                               const grid_map::Position& start_position) {
  if (path.size() <= 2 || !params->planner.astar.shortcut_path) {
    return path;
  }
  std::vector<Cell> shortcut;
  size_t current = 0;
  shortcut.push_back(path[current]);
  while (current + 1 < path.size()) {
    size_t next = current + 1;
    for (size_t candidate = path.size() - 1; candidate > current + 1; --candidate) {
      if (lineIsValid(search_map, params, path[current], path[candidate], start_position)) {
        next = candidate;
        break;
      }
    }
    shortcut.push_back(path[next]);
    current = next;
  }
  return shortcut;
}

std::vector<grid_map::Position> cellsToPositions(const SearchMap& search_map,
                                                 const std::vector<Cell>& cells) {
  std::vector<grid_map::Position> positions;
  positions.reserve(cells.size());
  for (const auto& cell : cells) {
    positions.push_back(cellPosition(search_map, cell));
  }
  return positions;
}

std::vector<grid_map::Position> filterAndResamplePositions(
    const std::vector<grid_map::Position>& input,
    const ParamsConstPtr& params) {
  if (input.size() <= 1) {
    return input;
  }

  std::vector<grid_map::Position> spaced;
  spaced.push_back(input.front());
  for (size_t i = 1; i + 1 < input.size(); ++i) {
    if (distance2d(spaced.back(), input[i]) >= params->planner.astar.min_point_spacing) {
      spaced.push_back(input[i]);
    }
  }
  spaced.push_back(input.back());

  std::vector<grid_map::Position> output;
  output.push_back(spaced.front());
  for (size_t i = 1; i < spaced.size(); ++i) {
    const auto& from = output.back();
    const auto& to = spaced[i];
    const double segment_length = distance2d(from, to);
    const int steps = std::max(1, static_cast<int>(
        std::ceil(segment_length / params->planner.astar.max_segment_length)));
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      output.emplace_back(from.x() + ratio * (to.x() - from.x()),
                          from.y() + ratio * (to.y() - from.y()));
    }
  }
  return output;
}

void appendState(const std::shared_ptr<GridAStarPlanner::StateSpace>& space,
                 const std::shared_ptr<Map>& map,
                 const ParamsConstPtr& params,
                 const grid_map::Position& position,
                 double yaw,
                 ompl::geometric::PathGeometric* path) {
  double xyzrpy[6];
  xyzrpy[0] = position.x();
  xyzrpy[1] = position.y();
  xyzrpy[2] = 0.0;
  xyzrpy[3] = 0.0;
  xyzrpy[4] = 0.0;
  xyzrpy[5] = yaw;

  if (map->isInside(position)) {
    try {
      map->get3DPoseFrom2D(xyzrpy);
      xyzrpy[5] = yaw;
    } catch (const std::exception&) {
      try {
        xyzrpy[2] = map->getHeightAtPosition(position);
      } catch (const std::exception&) {
        xyzrpy[2] = 0.0;
      }
    }
  }

  ompl::base::ScopedState<> state(space);
  auto state_se3 = state->as<GridAStarPlanner::StateType>();
  state_se3->setX(xyzrpy[0]);
  state_se3->setY(xyzrpy[1]);
  state_se3->setZ(xyzrpy[2]);
  setSO3FromRPY(state_se3->rotation(), xyzrpy + 3);
  path->append(state.get());
}

}  // namespace



GridAStarPlanner::GridAStarPlanner(const ParamsConstPtr& params,
                                   const std::shared_ptr<Map>& map,
                                   const std::shared_ptr<StateSpace>& space)
    : params_(params),
      map_(map),
      space_(space) {
}



PlannerStatus GridAStarPlanner::plan(const ompl::base::ScopedState<>& start,
                                     const ompl::base::ScopedState<>& goal,
                                     ompl::geometric::PathGeometric* path_out) const {
  if (!path_out) {
    return PlannerStatus::UNKNOWN;
  }

  const auto grid_map = map_->getMap();
  if (!grid_map.exists(params_->planner.elevation_layer)) {
    return PlannerStatus::NO_MAP;
  }
  if (!grid_map.exists("traversability_thresholded") &&
      !grid_map.exists(params_->planner.traversability_layer)) {
    return PlannerStatus::NO_MAP;
  }

  SearchMap search_map;
  search_map.map = grid_map;
  search_map.elevation_layer = params_->planner.elevation_layer;
  search_map.traversability_layer = params_->planner.traversability_layer;
  search_map.has_traversability_thresholded =
      search_map.map.exists("traversability_thresholded");
  search_map.has_observed = search_map.map.exists("observed");
  search_map.size_x = search_map.map.getSize().x();
  search_map.size_y = search_map.map.getSize().y();
  search_map.resolution = search_map.map.getResolution();

  const auto start_state = start->as<StateType>();
  const auto goal_state = goal->as<StateType>();
  const grid_map::Position start_position(start_state->getX(), start_state->getY());
  const grid_map::Position goal_position(goal_state->getX(), goal_state->getY());

  grid_map::Index start_index;
  grid_map::Index goal_index;
  if (!search_map.map.getIndex(start_position, start_index)) {
    return PlannerStatus::INVALID_START;
  }
  if (!search_map.map.getIndex(goal_position, goal_index)) {
    return PlannerStatus::INVALID_GOAL;
  }

  Cell start_cell{start_index.x(), start_index.y()};
  Cell goal_cell{goal_index.x(), goal_index.y()};
  if (!nearestValidCell(search_map,
                        params_,
                        start_cell,
                        start_position,
                        params_->planner.start_goal_search.start_radius,
                        &start_cell)) {
    return PlannerStatus::INVALID_START;
  }
  if (!nearestValidCell(search_map,
                        params_,
                        goal_cell,
                        start_position,
                        params_->planner.start_goal_search.goal_radius,
                        &goal_cell)) {
    return PlannerStatus::INVALID_GOAL;
  }

  const int node_count = search_map.size_x * search_map.size_y;
  std::vector<double> g_score(node_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(node_count, -1);
  std::vector<int> parent_direction(node_count, -1);
  std::vector<bool> closed(node_count, false);
  std::priority_queue<QueueNode> open;

  const int start_flat = toFlatIndex(start_cell, search_map.size_y);
  const int goal_flat = toFlatIndex(goal_cell, search_map.size_y);
  const auto goal_cell_position = cellPosition(search_map, goal_cell);
  const auto start_cell_position = cellPosition(search_map, start_cell);

  g_score[start_flat] = 0.0;
  open.push(QueueNode{start_flat, distance2d(start_cell_position, goal_cell_position)});

  static const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  unsigned int expansions = 0;
  bool found = false;
  while (!open.empty() && expansions < params_->planner.astar.max_expansions) {
    const auto current_node = open.top();
    open.pop();
    if (closed[current_node.index]) {
      continue;
    }
    closed[current_node.index] = true;
    ++expansions;

    if (current_node.index == goal_flat) {
      found = true;
      break;
    }

    const Cell current = fromFlatIndex(current_node.index, search_map.size_y);
    const auto current_position = cellPosition(search_map, current);

    for (int direction = 0; direction < 8; ++direction) {
      const Cell neighbor{current.x + dx[direction], current.y + dy[direction]};
      if (!isTraversableCell(search_map, params_, neighbor, start_position)) {
        continue;
      }
      const int neighbor_flat = toFlatIndex(neighbor, search_map.size_y);
      if (closed[neighbor_flat]) {
        continue;
      }

      const auto neighbor_position = cellPosition(search_map, neighbor);
      const double step_cost = distance2d(current_position, neighbor_position);
      const bool unknown = isUnknownCell(search_map, neighbor);
      const double trav = traversabilityValue(search_map, neighbor);
      double tentative_g = g_score[current_node.index] + step_cost;
      tentative_g += params_->planner.astar.traversability_weight *
                     (1.0 - trav) * step_cost;
      if (unknown) {
        tentative_g += params_->planner.astar.unknown_penalty * step_cost;
      }
      if (parent_direction[current_node.index] >= 0) {
        const double turn = std::abs(angleDiff(directionAngle(direction),
                                               directionAngle(parent_direction[current_node.index]))) / M_PI;
        tentative_g += params_->planner.astar.turn_weight * turn;
      }
      tentative_g += params_->planner.astar.line_bias_weight *
                     pointLineDistance(neighbor_position,
                                       start_cell_position,
                                       goal_cell_position);

      if (tentative_g < g_score[neighbor_flat]) {
        g_score[neighbor_flat] = tentative_g;
        parent[neighbor_flat] = current_node.index;
        parent_direction[neighbor_flat] = direction;
        const double h = distance2d(neighbor_position, goal_cell_position);
        open.push(QueueNode{neighbor_flat, tentative_g + h});
      }
    }
  }

  if (!found) {
    return PlannerStatus::NOT_SOLVED;
  }

  std::vector<Cell> cells;
  for (int cursor = goal_flat; cursor >= 0; cursor = parent[cursor]) {
    cells.push_back(fromFlatIndex(cursor, search_map.size_y));
    if (cursor == start_flat) {
      break;
    }
  }
  if (cells.empty() || toFlatIndex(cells.back(), search_map.size_y) != start_flat) {
    return PlannerStatus::NOT_SOLVED;
  }
  std::reverse(cells.begin(), cells.end());
  cells = shortcutPath(search_map, params_, cells, start_position);

  auto positions = filterAndResamplePositions(cellsToPositions(search_map, cells),
                                              params_);
  if (positions.empty()) {
    return PlannerStatus::NOT_SOLVED;
  }

  path_out->clear();
  const double final_yaw = getYawFromSO3(goal_state->rotation());
  for (size_t i = 0; i < positions.size(); ++i) {
    double yaw = final_yaw;
    if (i + 1 < positions.size()) {
      yaw = std::atan2(positions[i + 1].y() - positions[i].y(),
                       positions[i + 1].x() - positions[i].x());
    } else if (positions.size() > 1) {
      yaw = final_yaw;
    }
    appendState(space_, map_, params_, positions[i], yaw, path_out);
  }

  return PlannerStatus::SOLVED;
}
