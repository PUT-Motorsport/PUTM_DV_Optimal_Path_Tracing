#pragma once

#include "package_opt/Cones.h"
#include "package_opt/OptimalPath.h"
#include "visualization_msgs/MarkerArray.h"

namespace opt {
void on_cones_received_callback(package_opt::Cones::ConstPtr const &cones);
} //namespace opt
