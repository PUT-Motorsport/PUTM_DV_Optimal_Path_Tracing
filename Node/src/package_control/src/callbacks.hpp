#pragma once

#include <geometry_msgs/TwistStamped.h>

namespace Control {

double calculate_steering_angle() noexcept;
double calculate_throttle() noexcept;

void path_received_callback(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept;
void vehicle_speed_data_received_callback(
    geometry_msgs::TwistStamped::ConstPtr const &vehicle_speed_data) noexcept;
} // namespace Control
