#pragma once

/*
 * This file contains mocks of ros' data structures and methods to enable testing in any environment
 * */

#include <vector>
#include <string>

namespace package_opt::Cones {
  struct ConstPtr{
    std::vector<double> cones_left_x;
    std::vector<double> cones_left_y;
    std::vector<double> cones_right_x;
    std::vector<double> cones_right_y;
};
}

namespace package_opt {
    struct optimal_path {
        std::vector<double> optimal_path_x;
        std::vector<double> optimal_path_y;
    };
}

#define ROS_INFO(...) {printf(__VA_ARGS__);}

namespace ros {
    struct Publisher {
        char placeholder;
        void publish(package_opt::optimal_path const& optimalPath) {
            (void)optimalPath;
        }
    };
    struct NodeHandle {
        char placeholder;
        Publisher advertise(std::string const& str, int val) {
            (void)str;
            (void)val;
            return {0};
        }
    };
}