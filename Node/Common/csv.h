#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "../package_opt/src/common_defs.hpp"


namespace csv {

    template<typename T>
    bool dump(std::string const& filename, std::vector<opt::Point<T>> const &points) noexcept {
        std::fstream file(filename, std::ios::out);

        if (not file) {
            return false;
        }

        for (auto const &point: points) {
            file << point.x << "," << point.y << "\n";
        }
        std::flush(file);

        file.close();

        return true;
    }

    std::vector<opt::Point<double>> load2Dcoords(std::string const &filename) noexcept {
       std::fstream file(filename);
       std::vector<opt::Point<double>> loaded_points;
       if (not file) {
           return loaded_points;
       }

       std::string line;
       while (std::getline(file, line)) {
           const auto index = line.find(',');
           const auto x = std::stod(line.substr(0, index));
           const auto y = std::stod(line.substr(index + 1));
           loaded_points.emplace_back(opt::Point<double>{x, y});
       }

       return loaded_points;
    }
}