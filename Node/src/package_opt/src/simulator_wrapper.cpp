#include "simulator_wrapper.hpp"

#include "callbacks.hpp"
#include <ros/ros.h>
#include "package_opt/Cones.h"
#include "opt_assert.hpp"

namespace
{
    ros::Publisher track_visualization_publisher;
}

namespace opt::simulator
{

    [[maybe_unused]] void simulator_wrapper_init(ros::NodeHandle &node_handle)
    {
        track_visualization_publisher = node_handle.advertise<visualization_msgs::MarkerArray>("track_visualization", 3);
    }

    [[maybe_unused]] void on_cones_received_callback_simulator(visualization_msgs::MarkerArray::ConstPtr const &cones)
    {

        /*
            Translate the cones from the simulator to the format used by the optimal path tracing package.
        */

        ROS_INFO("Received cones from the simulator"); // sanity check
        package_opt::Cones::Ptr cones_msg(new package_opt::Cones);
        cones_msg->cones_x.reserve(cones->markers.size());
        cones_msg->cones_y.reserve(cones->markers.size());

        for (const auto &cone : cones->markers)
        {
            cones_msg->cones_x.push_back(cone.pose.position.x);
            cones_msg->cones_y.push_back(cone.pose.position.y);
        }

        opt::on_cones_received_callback(cones_msg);
    }

    [[maybe_unused]] void visualize_track(package_opt::OptimalPath const &path, std::vector<opt::Point<double>> rrt_path, 
        std::pair<double, double> goal, std::array<double, 3> const& color)
    {
        /*
            Visualize the track in the simulator. Translate arrays of coordinates to markers. 
        */
        ROS_INFO("Path length %d", (int)path.path_x.size());
        opt_assert(path.path_x.size() == path.path_y.size());
        visualization_msgs::MarkerArray track_visualization_markers;
        track_visualization_markers.markers.reserve(path.path_x.size() + 1);
        for (std::size_t iter = 0; iter < path.path_x.size(); ++iter)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "fsds/Lidar";
            marker.header.stamp = ros::Time::now();
            marker.ns = "track_visualization";
            marker.id = iter;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = path.path_x[iter];
            marker.pose.position.y = path.path_y[iter];
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
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            track_visualization_markers.markers.emplace_back(marker);
        }
        for(std::size_t iter = 0; iter < rrt_path.size(); ++iter) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "fsds/Lidar";
            marker.header.stamp = ros::Time::now();
            marker.ns = "track_visualization";
            marker.id = path.path_x.size() + iter;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = rrt_path.at(iter).x;
            marker.pose.position.y = rrt_path.at(iter).y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 0.;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            track_visualization_markers.markers.emplace_back(marker);
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "fsds/Lidar";
        marker.header.stamp = ros::Time::now();
        marker.ns = "track_visualization";
        marker.id = path.path_x.size() * 2 + 1;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = goal.first;
        marker.pose.position.y = goal.second;
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
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        // track_visualization_markers.markers.emplace_back(marker);
        track_visualization_publisher.publish(track_visualization_markers);
    }

}