#pragma once

#include <vector>

namespace package_opt {
namespace Cones {
  struct ConstPtr{
    std::vector<double> cones_left_x;
    std::vector<double> cones_left_y;
    std::vector<double> cones_right_x;
    std::vector<double> cones_right_y;
};
}
}

#define ROS_INFO(...) {printf(__VA_ARGS__);}
