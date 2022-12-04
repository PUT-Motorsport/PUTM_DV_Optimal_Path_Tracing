#pragma once

#define OPT_ROS_USE_MOCKS

#ifndef OPT_ROS_USE_MOCKS
#include "package_opt/Cones.h"
#else
#include "../../tests/mocks/ros_mocks.hpp"
#endif
namespace opt{

void on_cones_received_callback(package_opt::Cones::ConstPtr const &cones) noexcept;

} //namespace opt
