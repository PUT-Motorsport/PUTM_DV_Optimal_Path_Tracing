#include "callbacks.hpp"

#include <package_control/ControlCommand.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <cmath>

#include "config.hpp"
#include "lateral.hpp"
#include "longitudinal.hpp"
/*
 * The steering angle is calculated immediately after receiving new path.
 * The throttle position is calculated at 10 Hz, and the velocity data is stored
 * to be available on request. The reasoning is that the steering data will be
 * published more often that it is needed, while the path may be published
 * slower than 10 Hz.
 */
namespace {
double vehicle_velocity;
double steering_angle;

bool path_received;

}  // namespace

namespace Control {

double calculate_steering_angle() noexcept { return steering_angle; }

double calculate_throttle() noexcept {
  if (not path_received) {
    return 0.0;
  }

  if (vehicle_velocity > Config::target_velocity) {
    return 0.0;
  }

  return 1.0;
}

void path_received_callback(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept {
  ROS_INFO("Path received");
  path_received = true;
  std::optional angle_opt = pure_pursuit_get_angle<double>(optimal_path);

  if (not angle_opt.has_value())
    return;  // if it's not possible to compute the new angle, the prevoius
             // angle remains

  const auto angle_rad = angle_opt.value();
  const auto angle_deg = opt::utility::rad_to_deg(angle_rad);
  const auto clamped_angle = std::clamp(
      angle_deg, -1 * Config::max_steering_angle, Config::max_steering_angle);
  steering_angle = clamped_angle / Config::max_steering_angle;
}

void vehicle_speed_data_received_callback(
    geometry_msgs::TwistStamped::ConstPtr const &vehicle_speed_data) noexcept {
  // calculate the vehicle velocity in xy plane
  vehicle_velocity = std::sqrt(std::hypot(vehicle_speed_data->twist.linear.x,
                                          vehicle_speed_data->twist.linear.y));
}
}  // namespace Control
