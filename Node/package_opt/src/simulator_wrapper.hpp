#pragma once

#include <package_opt/OptimalPath.h>
#include <ros/ros.h>

#include <utility>  //pair

#include "common_defs.hpp"
#include "visualization_msgs/MarkerArray.h"

namespace opt::simulator {
[[maybe_unused]] void simulator_wrapper_init(ros::NodeHandle &node_handle);
[[maybe_unused]] void on_cones_received_callback_simulator(
    visualization_msgs::MarkerArray::ConstPtr const &cones);
[[maybe_unused]] void visualize_track(package_opt::OptimalPath const &path,
                                      std::vector<opt::Point<double>> rrt_path,
                                      std::pair<double, double> goal,
                                      std::array<double, 3> const &color);
}  // namespace opt::simulator
