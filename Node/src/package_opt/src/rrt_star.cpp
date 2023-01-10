#include "rrt_star.hpp"

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>

#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

#include "validity_checker.hpp"
#include "path.hpp"
#include "config.hpp"

namespace opt {

std::pair<std::vector<double>, std::vector<double>> get_rrt_path(package_opt::Cones::ConstPtr const &cones) {
        opt_assert(cones->cones_x.size() == cones->cones_y.size());

        const auto min_x = *std::min_element(cones->cones_x.begin(), cones->cones_x.end());
        const auto min_y = *std::min_element(cones->cones_y.begin(), cones->cones_y.end());
        auto lower_space_bound = (min_x < min_y) ? min_x : min_y;
        ROS_INFO("Min x: %f y: %f", min_x, min_y);

        const auto max_x = *std::max_element(cones->cones_x.begin(), cones->cones_x.end());
        const auto max_y = *std::max_element(cones->cones_y.begin(), cones->cones_y.end());
        auto upper_space_bound = (max_x > max_y) ? max_x : max_y;
        ROS_INFO("Max x: %f y: %f", max_x, max_y);
        ROS_INFO("Upper: %f lower %f", upper_space_bound, lower_space_bound);

        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
        // this will create a state with bounds [lower_space_bound, upper_space_bound] in each dimension
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(lower_space_bound, upper_space_bound);
        ompl::base::SpaceInformationPtr space_information(new ompl::base::SpaceInformation(space));
        space_information->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new opt::ValidityChecker<double>(space_information, cones)));
        space_information->setup();

        ompl::base::ScopedState start(space);
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0.0; // todo: will break after shifting to global coordinates
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0.0;
        ompl::base::ScopedState goal(space);
        opt_assert(cones->cones_x.size() >= 2);
        
        const auto goal_x = (cones->cones_x.at(config::max_cones_horizon) + cones->cones_x.at(config::max_cones_horizon + 1)) / 2;  //fixme: deciding goal position: assumptions may not always hold
        const auto goal_y = (cones->cones_y.at(config::max_cones_horizon) + cones->cones_y.at(config::max_cones_horizon + 1)) / 2;
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = goal_x;
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = goal_y;

        ompl::base::ProblemDefinitionPtr problem_definition(new ompl::base::ProblemDefinition(space_information));
        problem_definition->setStartAndGoalStates(start, goal);
        problem_definition->setOptimizationObjective(opt::getPathLengthObjective(space_information));

        ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(space_information));

        planner->setProblemDefinition(problem_definition);
        planner->setup();

        const ompl::base::PlannerStatus solved = planner->solve(config::max_calculation_time);

        if (not solved)
        {
            ROS_INFO("Optimal path tracing package failed to solve for RRT* path in its time quanta of %f", config::max_calculation_time);
            return std::pair{std::vector<double>(), std::vector<double>()};
        }

        const auto states = problem_definition->getSolutionPath()->as<ompl::geometric::PathGeometric>()->getStates();

        std::vector<double> rrt_path_x;
        std::vector<double> rrt_path_y;
        rrt_path_x.reserve(states.size());
        rrt_path_y.reserve(states.size());
        for (std::size_t iter = 0; iter < states.size(); ++iter)
        {
            rrt_path_x.emplace_back(states.at(iter)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]);
            rrt_path_y.emplace_back(states.at(iter)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]);
        }
        return std::pair{rrt_path_x, rrt_path_y};

}
} //namespace opt
