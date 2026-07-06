#pragma once

#include <memory>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include "art_planner/map/map.h"
#include "art_planner/params.h"
#include "art_planner/planner_status.h"



namespace art_planner {



class GridAStarPlanner {

public:
  using StateSpace = ompl::base::SE3StateSpace;
  using StateType = StateSpace::StateType;

  GridAStarPlanner(const ParamsConstPtr& params,
                   const std::shared_ptr<Map>& map,
                   const std::shared_ptr<StateSpace>& space);

  PlannerStatus plan(const ompl::base::ScopedState<>& start,
                     const ompl::base::ScopedState<>& goal,
                     ompl::geometric::PathGeometric* path_out) const;

private:
  ParamsConstPtr params_;
  std::shared_ptr<Map> map_;
  std::shared_ptr<StateSpace> space_;

};



}
