#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

#include "callbacks.hpp"
#include "config.hpp"
#include "longitudinal.hpp"
#include "package_control/ControlCommand.h"

int main(int argc, char **argv) {
  ROS_INFO("Initializing package_control");
  ros::init(argc, argv, "package_control");
  ros::NodeHandle node_handle;

  auto control_command_publisher =
      node_handle.advertise<package_control::ControlCommand>(
          "/fsds/control_command", 3);

  auto optimal_path_subscriber = node_handle.subscribe(
      "/OptimalPath", 3, Control::path_received_callback);

  ros::Rate control_loop_rate{
      Control::Config::longitudinal_control_loop_frequency};

  while (ros::ok()) {
    static int time = 0;
    double throttle = 0.0;
    if (time++ < 20) {
      throttle = 0.3;
    }

    package_control::ControlCommand control_command{};
    control_command.throttle = throttle; //Control::calculate_throttle();
    control_command.steering = Control::calculate_steering_angle();
    control_command.brake = 0.0;

    control_command_publisher.publish(control_command);

    ros::spinOnce();
    control_loop_rate.sleep();
  }
}
