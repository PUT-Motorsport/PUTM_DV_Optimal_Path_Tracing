#pragma once

#include <ros/ros.h>

namespace control::simulator {
[[maybe_unused]] void simulator_wrapper_init(ros::NodeHandle& node_handle);
[[maybe_unused]] void visualize_direction(double x_lookhead, double y_lookahead,
                                          double angle);
}  // namespace control::simulator