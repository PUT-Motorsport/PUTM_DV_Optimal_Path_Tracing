#pragma once

#include "config.hpp"
#include "math_utility.hpp"
#include "opt_assert.hpp"
#include <optional>
#include <package_opt/OptimalPath.h>
#include <ros/ros.h>

#include "visualization.hpp"

namespace Control {

template <typename T>
[[maybe_unused]] constexpr double stanley_get_angle(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept;

template <typename T>
[[maybe_unused]] std::optional<T> pure_pursuit_get_angle(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept {
  static_assert(std::is_floating_point_v<T>);
  for (std::size_t iter = 1; iter < optimal_path->path_x.size(); ++iter) {

    const T distance = std::hypot(optimal_path->path_x.at(iter),
                                  optimal_path->path_y.at(iter));

    if (distance > Config::lookahead_distance) {
      opt_assert(optimal_path->path_x.size() >= 2);
      const auto x_lookahead =
          (optimal_path->path_x.at(iter) + optimal_path->path_x.at(iter - 1)) /
          2; // fixme
      const auto y_lookahead =
          (optimal_path->path_y.at(iter) + optimal_path->path_y.at(iter - 1)) /
          2;
      ROS_INFO("Lookahead. x: %f, y: %f", x_lookahead, y_lookahead);
#ifndef NDEBUG
      control::simulator::visualize_direction(x_lookahead, y_lookahead, 0.00);
#endif
      return std::optional<T>{std::atan2(y_lookahead, x_lookahead)};
    }
  }
  ROS_WARN("No points on path met conditions.");
  return std::nullopt;
}

} // namespace Control
