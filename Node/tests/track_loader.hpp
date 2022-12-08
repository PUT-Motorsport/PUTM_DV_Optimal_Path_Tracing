#pragma once

#include "mocks/ros_mocks.hpp"
#include <cassert>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <iostream>
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

    namespace {
        std::vector<std::string> split_csv_line(std::string const & line) {
            std::stringstream stream(line);
            std::vector<std::string> parsed_line;
            while (stream.good()) {
                std::string substring;
                std::getline(stream, substring, ',');
                parsed_line.emplace_back(std::move(substring));
            }
            return parsed_line;
        }
        double string_to_double(std::string const &str) {
            std::size_t len{};
            double result = std::stod(str, &len);
            if (len == 0) {
                std::cerr << "Bad string parsing. Str: " << str << std::endl;
            }
            return result;
        }
    }

    package_opt::Cones::ConstPtr load_track_file(std::string const &filename) {
        //this method expects the file to contain four rows with left_x, left_y, right_x, right_y positions
        //one line of column headers is also expected
        std::fstream file;
        file.open(filename);
        std::string line;
        std::getline(file, line);   //skip the header

        package_opt::Cones::ConstPtr cones;

        while (not file.eof()) {
            std::getline(file, line);
            const auto split_line = split_csv_line(line);
            assert(split_line.size() == 4);
            cones.cones_left_x.emplace_back(string_to_double(split_line.at(0)));
            cones.cones_left_y.emplace_back(string_to_double(split_line.at(1)));
            cones.cones_right_x.emplace_back(string_to_double(split_line.at(2)));
            cones.cones_right_y.emplace_back(string_to_double(split_line.at(3)));

        }
        return cones;
    }

} //namespace package_opt::testing
