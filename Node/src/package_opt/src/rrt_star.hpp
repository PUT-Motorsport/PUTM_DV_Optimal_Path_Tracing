#pragma once

#include <vector>
#include <utility>

#include "package_opt/Cones.h"

namespace opt {

std::pair<std::vector<double>, std::vector<double>> get_rrt_path(package_opt::Cones::ConstPtr const &cones);

} //namespace opt
