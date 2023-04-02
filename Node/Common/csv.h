#pragma once

#include <fstream>
#include <string>

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
}