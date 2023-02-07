#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

#include "callbacks.hpp"
#include "config.hpp"
#include "longitudinal.hpp"
#include "package_control/ControlCommand.h"

#ifndef NDEBUG
#include "visualization.hpp"
#endif

int main(int argc, char **argv) {
  ROS_INFO("Initializing package_control");
  ros::init(argc, argv, "package_control");
  ros::NodeHandle node_handle;

  auto control_command_publisher =
      node_handle.advertise<package_control::ControlCommand>(
          "/fsds/control_command", 3);

  auto optimal_path_subscriber =
      node_handle.subscribe("/OptimalPath", 3, Control::path_received_callback);

  auto speed_data_subscriber = node_handle.subscribe(
      "/fsds/gss", 3, Control::vehicle_speed_data_received_callback);

#ifndef NDEBUG
  control::simulator::simulator_wrapper_init(node_handle);
#endif

  ros::Rate control_loop_rate{
      Control::Config::longitudinal_control_loop_frequency};

  while (ros::ok()) {

    package_control::ControlCommand control_command{};
    control_command.throttle = Control::calculate_throttle();
    control_command.steering =
        -1 *
        Control::calculate_steering_angle(); // positive angle is to the right
    control_command.brake = 0.0;

    control_command_publisher.publish(control_command);

    ros::spinOnce();
    control_loop_rate.sleep();
  }
}
