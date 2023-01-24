#pragma once

#include "config.hpp"
#include <package_opt/OptimalPath.h>

namespace Control {
[[maybe_unused]] constexpr double stanley_get_angle(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept;
[[maybe_unused]] double pure_pursuit_get_angle(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept;
constexpr double steering_angle_to_fraction(double steering_angle) noexcept;
} // namespace Control
