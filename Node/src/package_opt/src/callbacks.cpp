#include "callbacks.hpp"
#include "validity_checker.hpp"
#include "path.hpp"

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>

#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

#include <type_traits>
#include <algorithm>
#include <chrono>
#include <utility>

// todo: define symbol in cmake
#define SIMULATOR

#ifdef SIMULATOR
#include "simulator_wrapper.hpp"
#endif

namespace opt
{
    extern ros::Publisher optimal_path_publisher;

    void on_cones_received_callback(package_opt::Cones::ConstPtr const &cones)
    {
        opt_assert(cones->cones_x.size() == cones->cones_y.size());

        const auto start_time = std::chrono::steady_clock::now();
        // todo: change to fixed heuristics instead of calculating bounds?
        const auto min_x = *std::min(cones->cones_x.begin(), cones->cones_x.end());
        const auto min_y = *std::min(cones->cones_y.begin(), cones->cones_y.end());
        auto lower_space_bound = (min_x < min_y) ? min_x : min_y;

        const auto max_x = *std::max(cones->cones_x.begin(), cones->cones_x.end());
        const auto max_y = *std::max(cones->cones_y.begin(), cones->cones_y.end());
        auto upper_space_bound = (max_x > max_y) ? max_x : max_y;
        ROS_INFO("Upper: %f lower %f", upper_space_bound, lower_space_bound);

        lower_space_bound = -30;
        upper_space_bound = 30;

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

        // todo: find and implement optimality threshold

        const ompl::base::PlannerStatus solved = planner->solve(config::max_calculation_time);

        if (not solved)
        {
            ROS_INFO("Optimal path tracing package failed to solve for RRT* path in its time quanta of %f", config::max_calculation_time);
            return;
        }

        const auto states = problem_definition->getSolutionPath()->as<ompl::geometric::PathGeometric>()->getStates();

        std::vector<double> rrt_path_x(states.size());
        std::vector<double> rrt_path_y(states.size());
        for (std::size_t iter = 0; iter < rrt_path_x.size(); ++iter)
        {
            rrt_path_x.at(iter) = states.at(iter)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            rrt_path_y.at(iter) = states.at(iter)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
        }

        // const auto [smooth_path_x, smooth_path_y] = path::rrt_path_smoothing(rrt_path_x, rrt_path_y);

        package_opt::OptimalPath optimalPath;
        optimalPath.path_x = std::move(rrt_path_x);
        optimalPath.path_y = std::move(rrt_path_y);

        optimal_path_publisher.publish(optimalPath);
        const auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time);
        ROS_INFO("Optimal path traced in %ld ms. States: %d", time_diff.count(), (int)states.size());

#ifdef SIMULATOR
        simulator::visualize_track(optimalPath, std::make_pair(goal_x, goal_y));
#endif
    }
} // namespace opt