#include "visualization.hpp"

#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

namespace {
    ros::Publisher control_visualization_publisher;
}

namespace control::simulator {
    [[maybe_unused]] void simulator_wrapper_init(ros::NodeHandle& node_handle) {
        control_visualization_publisher = node_handle.advertise<visualization_msgs::Marker>("direction", 3);
    }
    [[maybe_unused]] void visualize_direction(double x_lookhead, double y_lookahead, double angle) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "fsds/Lidar";
        marker.header.stamp = ros::Time::now();
        marker.ns = "track_visualization";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x_lookhead;
        marker.pose.position.y = y_lookahead;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        
        control_visualization_publisher.publish(marker);
    }
}