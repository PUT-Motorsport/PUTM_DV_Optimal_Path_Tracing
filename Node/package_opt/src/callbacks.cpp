#include "callbacks.hpp"

#include <algorithm>
#include <chrono>
#include <type_traits>
#include <utility>

#include <ros/ros.h>
#include "Delaunay/delaunay.h"
#include "config.hpp"
#include "opt_assert.hpp"
#include "rrt_star.hpp"
#include "delaunay_optimization.hpp"
// todo: define symbol in cmake
#define SIMULATOR

#ifdef SIMULATOR
#include "simulator_wrapper.hpp"
#endif

namespace opt {
extern ros::Publisher optimal_path_publisher;

void on_cones_received_callback(package_opt::Cones::ConstPtr const &cones) {
  const auto algorithm_start_time = std::chrono::steady_clock::now();
  opt_assert(cones->cones_x.size() == cones->cones_y.size());

  const auto rrt_start_time = std::chrono::steady_clock::now();
  const auto rrt_path = get_rrt_path(cones);
  const auto time_diff_rrt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - rrt_start_time);
  ROS_INFO("RRT* took %ld ms", time_diff_rrt.count());
  const auto delaunay_path = delaunay_optimize_track(cones, rrt_path);

  package_opt::OptimalPath optimalPath;
  optimalPath.path_x.reserve(delaunay_path.size());
  optimalPath.path_y.reserve(delaunay_path.size());

  for (std::size_t iter = 0; iter < delaunay_path.size(); ++iter) {
    optimalPath.path_x.emplace_back(delaunay_path.at(iter).x);
    optimalPath.path_y.emplace_back(delaunay_path.at(iter).y);
  }
  optimal_path_publisher.publish(optimalPath);


  const auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - algorithm_start_time);
  ROS_INFO("Optimal path traced in %ld ms. States: %d", time_diff.count(),
           (int)optimalPath.path_x.size());

#ifdef SIMULATOR
    double goal_x, goal_y;
    std::size_t count;

    for (std::size_t iter = 0; iter < cones->cones_x.size(); ++iter) {
        const auto distance = std::hypot(cones->cones_x.at(iter), cones->cones_y.at(iter));
        if (distance < config::cone_threshold_high and distance > config::cone_threshold_low) {
            count++;
            goal_x += cones->cones_x.at(iter);
            goal_y += cones->cones_y.at(iter);
        }
    }
    opt_assert(goal_y not_eq 0.0);
    opt_assert(goal_x not_eq 0.0);

    goal_x /= (double)count;
    goal_y /= (double)count;
  simulator::visualize_track(optimalPath, rrt_path, std::make_pair(goal_x, goal_y), std::array<double, 3>{1.0, 0.0, 0.0});
#endif
}
} // namespace opt
