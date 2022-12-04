#include <vector>
#include "mocks/ros_mocks.hpp"
#include "callbacks.h"

int main(int argc, char ** argv) {
  //Load data from a test source and put it through the algorithm
  std::vector<double> cones_left_x {1.0, 2.0, 3.0, 4.0};
  std::vector<double> cones_left_y {2.0, 3.0, 4.0, 5.0};
  std::vector<double> cones_right_x {3.0, 4.0, 5.0, 6.0};
  std::vector<double> cones_right_y {4.0, 5.0, 6.0, 7.0};
  package_opt::Cones::ConstPtr cones{
      .cones_left_x = cones_left_x,
      .cones_left_y = cones_left_y,
      .cones_right_x = cones_right_x,
      .cones_right_y = cones_right_y
  };

    opt::on_cones_received_callback(cones);

  return 0;
}
