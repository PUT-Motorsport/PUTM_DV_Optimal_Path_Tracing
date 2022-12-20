#include "callbacks.hpp"
#include "validity_checker.hpp"
#include "path.hpp"

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

#include <type_traits>
#include <algorithm>
#include <chrono>

namespace opt {

    extern ros::Publisher optimal_path_publisher;

void on_cones_received_callback(package_opt::Cones::ConstPtr const &cones) {
    const auto start_time = std::chrono::steady_clock::now();
    //todo: change to fixed heuristics instead of calculating bounds?
    const double lower_space_bound = std::min(
            std::min(*std::min_element(cones->cones_left_x.begin(), cones->cones_left_x.end()),
                                        *std::min_element(cones->cones_right_x.begin(), cones->cones_right_x.end())),
            std::min(*std::min_element(cones->cones_left_y.begin(), cones->cones_left_y.end()),
                     *std::min_element(cones->cones_right_y.begin(), cones->cones_right_y.end())));

    const double upper_space_bound = std::max(
            std::max(*std::max_element(cones->cones_left_x.begin(), cones->cones_left_x.end()),
                     *std::max_element(cones->cones_right_x.begin(), cones->cones_right_x.end())),
            std::max(*std::max_element(cones->cones_left_y.begin(), cones->cones_left_y.end()),
                     *std::max_element(cones->cones_right_y.begin(), cones->cones_right_y.end())));


  ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(lower_space_bound, upper_space_bound);
  ompl::base::SpaceInformationPtr space_information(new ompl::base::SpaceInformation(space));
  space_information->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new opt::ValidityChecker<double>(space_information, cones)));
  space_information->setup();

  ompl::base::ScopedState start(space);
  start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0.0;
  start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0.0;
  ompl::base::ScopedState goal(space);
  goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = (cones->cones_left_x.back() + cones->cones_right_x.back()) / 2;
  goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = (cones->cones_left_y.back() + cones->cones_right_y.back()) / 2;

  ompl::base::ProblemDefinitionPtr problem_definition(new ompl::base::ProblemDefinition(space_information));
  problem_definition->setStartAndGoalStates(start, goal);
  problem_definition->setOptimizationObjective(opt::getPathLengthObjective(space_information));

  ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(space_information));

  planner->setProblemDefinition(problem_definition);
  planner->setup();

  //todo: find and implement optimality threshold

  const ompl::base::PlannerStatus solved = planner->solve(config::max_calculation_time);

  if (not solved) {
      ROS_INFO("Optimal path tracing package failed to solve for RRT* path in its time quanta of %f", config::max_calculation_time);
      return;
  }

  const auto states = problem_definition->getSolutionPath()->as<ompl::geometric::PathGeometric>()->getStates();

  std::vector<double> rrt_path_x(states.size());
  std::vector<double> rrt_path_y(states.size());

  for (std::size_t iter = 0; iter < rrt_path_x.size(); ++iter) {
      rrt_path_x.at(iter) = states.at(iter)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
      rrt_path_y.at(iter) = states.at(iter)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
  }

  const auto [smooth_path_x, smooth_path_y] = path::rrt_path_smoothing(rrt_path_x, rrt_path_y);

  package_opt::OptimalPath optimalPath;
  optimalPath.path_x = std::move(smooth_path_x);
  optimalPath.path_y = std::move(smooth_path_y);

  optimal_path_publisher.publish(optimalPath);
  const auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time);
  ROS_INFO("Optimal path traced in %ld ms", time_diff.count());

  //todo: rrt*'s path smoothing?
}
} //namespace opt
