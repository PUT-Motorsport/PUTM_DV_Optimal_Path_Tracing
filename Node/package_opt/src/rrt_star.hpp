#pragma once

#include <utility>
#include <vector>

#include "common_defs.hpp"
#include "package_opt/Cones.h"

namespace opt {

std::vector<opt::Point<double>> get_rrt_path(
    package_opt::Cones::ConstPtr const &cones);

}  // namespace opt
