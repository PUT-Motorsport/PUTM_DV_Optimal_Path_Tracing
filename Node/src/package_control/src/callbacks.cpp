#include "callbacks.hpp"

#include <ros/console.h>
#include <ros/ros.h>

#include <package_control/ControlCommand.h>

#include <cmath>

#include "config.hpp"
#include "lateral.hpp"
#include "longitudinal.hpp"
/*
 * The steering angle is calculated immediately after receiving new path.
 * The throttle position is calculated at 10 Hz, and the velocity data is stored
 * to be available on request. The reasoning is that the steering data will be
 * published more often that it is needed, while the path may be publisher
 * slower than 10 Hz.
 */
namespace {
double vehicle_velocity;
double steering_angle;
} // namespace

namespace Control {

double calculate_steering_angle() noexcept { return steering_angle; }

double calculate_throttle() noexcept {
  using namespace Control::Config;
  static Control::PID<double> velocity_pid(
      Ki, Kp, Kd, (1. / longitudinal_control_loop_frequency));

  const auto throttle =
      velocity_pid.update(Control::Config::target_velocity, vehicle_velocity);

  return throttle;
}

void path_received_callback(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept {
  ROS_INFO("Path received");
  steering_angle = pure_pursuit_get_angle(optimal_path);
}

void vehicle_speed_data_received_callback(
    geometry_msgs::TwistStamped::ConstPtr const &vehicle_speed_data) noexcept {
  ROS_INFO("Vehicle velocity received");
  // calculate the vehicle velocity in xy plane
  vehicle_velocity = std::sqrt(std::hypot(vehicle_speed_data->twist.linear.x,
                                          vehicle_speed_data->twist.linear.y));
}
} // namespace Control
