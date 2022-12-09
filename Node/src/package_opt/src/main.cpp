#include <ros/ros.h>
#include "package_opt/Cones.h"
#include "package_opt/Optimal_Path.h"
#include "callbacks.hpp"

namespace opt {
    ros::Publisher optimal_path_publisher;
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "package_opt");
  ros::NodeHandle node_handle;

  opt::optimal_path_publisher = node_handle.advertise<package_opt::Cones>("Optimal_Path", 3);
  auto subscriber = node_handle.subscribe("Cones", 3, opt::on_cones_received_callback);

  ros::spin();
}
