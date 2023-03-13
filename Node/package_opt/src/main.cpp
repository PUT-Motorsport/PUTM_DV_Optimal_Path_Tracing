#define SIMULATOR

#include <ros/ros.h>

#include "callbacks.hpp"
#include "package_opt/Cones.h"
#include "simulator_wrapper.hpp"
#include "visualization_msgs/MarkerArray.h"

namespace opt {
ros::Publisher optimal_path_publisher;
}

int main(int argc, char** argv) {
  ROS_INFO("Initializing package_opt_node");
  ros::init(argc, argv, "package_opt");
  ros::NodeHandle node_handle;

  opt::optimal_path_publisher =
      node_handle.advertise<package_opt::OptimalPath>("OptimalPath", 3);
  auto subscriber =
      node_handle.subscribe("Cones", 3, opt::on_cones_received_callback);

#ifdef SIMULATOR
  opt::simulator::simulator_wrapper_init(node_handle);
  auto subscriber_sim = node_handle.subscribe<visualization_msgs::MarkerArray>(
      "cones_clusters", 3,
      opt::simulator::on_cones_received_callback_simulator);
#endif

  ros::spin();
}
