#pragma once

#include "mocks/ros_mocks.hpp"
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
/*
 * Code to load tracks from .csv's into four vectors of cone positions
 * */

namespace package_opt::testing {

    std::vector<std::string> find_all_track_files(std::string const &relative_path = "\\tracks") {
        std::vector<std::string> csvFiles;
        std::filesystem::path file_path{relative_path};
        for (const auto &file: std::filesystem::directory_iterator(file_path))
        {
            std::string path = file.path().string();
            if (path.find(".csv") not_eq std::string::npos){
                csvFiles.emplace_back(file.path().string());
            }
        }
        return csvFiles;
    }

    package_opt::Cones::ConstPtr load_track_file(std::string const &filename) {
        //this method expects the file to contain four rows with left_x, left_y, right_x, right_y positions
        //one line of column headers is also expected
        std::fstream file;
        file.open(filename);
    }

} //namespace package_opt::testing
