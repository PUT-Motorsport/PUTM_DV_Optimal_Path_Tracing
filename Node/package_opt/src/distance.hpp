#pragma once

#include <type_traits>
#include <cmath>

namespace opt::utility {

    template<typename T>
    requires(std::is_arithmetic<T>::value)
    constexpr T L2_distance(T x1, T x2, T y1, T y2) noexcept {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }

} //namespace opt::utility
