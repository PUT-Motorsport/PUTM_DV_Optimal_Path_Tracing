#include <vector>
#include "mocks/ros_mocks.hpp"
#include "callbacks.h"
#include "track_loader.hpp"

/*
 * This file links the Node code with mocks and (will) run tests to evaluate the node performance
 * */

ros::Publisher optimal_path_publisher;

int main(int argc, char ** argv) {
  //Load data from a test source and put it through the algorithm

  std::vector<std::string> track_files{package_opt::testing::find_all_track_files()};

  for (const auto &track_file_name : track_files) {
      package_opt::Cones::ConstPtr cones{package_opt::testing::load_track_file(track_file_name)};
      opt::on_cones_received_callback(cones);

  }

  return 0;
}
