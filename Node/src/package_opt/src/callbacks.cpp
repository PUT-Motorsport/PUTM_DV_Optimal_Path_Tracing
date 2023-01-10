#include "callbacks.hpp"

#include <type_traits>
#include <algorithm>
#include <chrono>
#include <utility>

#include "rrt_star.hpp"
#include "Delaunay/delaunay.h"
#include "spline_interpolation.hpp"
#include "config.hpp"
#include "opt_assert.hpp"

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
        const auto algorithm_start_time = std::chrono::steady_clock::now();
        opt_assert(cones->cones_x.size() == cones->cones_y.size());
        //Delanuay
        std::vector<dt::Vector2<double>> points;
        points.reserve(cones->cones_x.size());
        for (std::size_t i = 0; i < cones->cones_x.size(); ++i)
        {
            points.emplace_back(cones->cones_x[i], cones->cones_y[i]);
        }
        const auto delanuay_start_time = std::chrono::steady_clock::now();
        dt::Delaunay<double> triangulation;
        triangulation.triangulate(points);
        const auto& triangles = triangulation.getTriangles();
        const auto time_diff_delanuay = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - delanuay_start_time);
        ROS_INFO("Delaunay triangulation done in %ld ms. Triangles: %d", time_diff_delanuay.count(), (int)triangles.size());
        
        
        const auto [rrt_path_x, rrt_path_y] = get_rrt_path(cones);
        const auto [extrapolated_track_x, extrapolated_track_y] = track_spline_interpolation(rrt_path_x, rrt_path_y);
        for (std::size_t iter = 0; iter < extrapolated_track_x.size(); ++iter) {
            ROS_INFO("x: %f \t y: %f", extrapolated_track_x, extrapolated_track_y);
        }

        package_opt::OptimalPath optimalPath;
        optimalPath.path_x = std::move(rrt_path_x);
        optimalPath.path_y = std::move(rrt_path_y);

        optimal_path_publisher.publish(optimalPath);
        const auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - algorithm_start_time);
        ROS_INFO("Optimal path traced in %ld ms. States: %d", time_diff.count(), (int)optimalPath.path_x.size());

#ifdef SIMULATOR
        const auto goal_x = (cones->cones_x.at(opt::config::max_cones_horizon) + cones->cones_x.at(opt::config::max_cones_horizon + 1)) / 2;  //fixme: deciding goal position: assumptions may not always hold
        const auto goal_y = (cones->cones_y.at(opt::config::max_cones_horizon) + cones->cones_y.at(opt::config::max_cones_horizon + 1)) / 2;
        simulator::visualize_track(optimalPath, std::make_pair(goal_x, goal_y));
#endif
    }
} // namespace opt
