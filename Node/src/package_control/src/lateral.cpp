#include "lateral.hpp"

#include <algorithm>
#include <cmath>

namespace Control {
constexpr double stanley_get_angle(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept {
  return 0;
}

constexpr double pure_pursuit_get_angle(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept {
  // find point on path closest to lookahead distance
  double angle;
  for (std::size_t iter = 0; iter < optimal_path->path_x.size(); ++iter) {
    const auto x = optimal_path->path_x[iter];
    const auto y = optimal_path->path_y[iter];
    const auto distance = std::hypot(x, y);
    if (distance > Config::lookahead_distance) {
      const auto x_prev = optimal_path->path_x[iter - 1];
      const auto y_prev = optimal_path->path_y[iter - 1];
      const auto distance_prev = std::hypot(x_prev, y_prev);
      const auto distance_diff = distance - distance_prev;
      const auto x_diff = x - x_prev;
      const auto y_diff = y - y_prev;
      const auto x_lookahead =
          x_prev +
          (Config::lookahead_distance - distance_prev) / distance_diff * x_diff;
      const auto y_lookahead =
          y_prev +
          (Config::lookahead_distance - distance_prev) / distance_diff * y_diff;
      angle = std::atan2(y_lookahead, x_lookahead);
    }
  }
  return angle;
}

constexpr double steering_angle_to_fraction(double steering_angle) noexcept {
  const auto fraction = steering_angle / Config::max_steering_angle;
  return std::clamp(fraction, -1.0, 1.0);
}
} // namespace Control
