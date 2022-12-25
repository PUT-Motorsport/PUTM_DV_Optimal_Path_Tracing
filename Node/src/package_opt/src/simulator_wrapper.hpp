#pragma once

#include "visualization_msgs/MarkerArray.h"
#include <ros/ros.h>
#include <package_opt/OptimalPath.h>

#include <utility> //pair

namespace opt::simulator {
    [[maybe_unused]] void simulator_wrapper_init(ros::NodeHandle &node_handle);
    [[maybe_unused]] void on_cones_received_callback_simulator(visualization_msgs::MarkerArray::ConstPtr const &cones);
    [[maybe_unused]] void visualize_track(package_opt::OptimalPath const &path, std::pair<double, double> goal);
}