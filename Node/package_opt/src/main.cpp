#include <ros/ros.h>
#include "callbacks.h"

int main(int argc, char ** argv) {
  ros::init(argc, argv, "Optimal_Path_Tracing");

  ros::NodeHandle node_handle;
  ros::Subscriber subscriber = node_handle.subscribe("cones", 3, opt::on_cones_received_callback); 

  ros::spin();
}
