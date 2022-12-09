#pragma once

#include "package_opt/Cones.h"

namespace opt {
void on_cones_received_callback(package_opt::Cones::ConstPtr const &cones);
} //namespace opt
