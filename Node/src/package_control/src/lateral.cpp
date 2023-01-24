#include "lateral.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

#include "math_utility.hpp"

namespace Control {
constexpr double stanley_get_angle(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept {
  return 0;
}

template <typename T>
T pure_pursuit_get_angle(
    package_opt::OptimalPath::ConstPtr const &optimal_path) noexcept {
  static_assert(std::is_floating_point_v<T>);
  for (std::size_t iter = 0; iter < optimal_path->path_x.size(); ++iter) {

    const T distance = std::hypot(optimal_path->path_x.at(iter),
                                  optimal_path->path_y.at(iter));

    if (opt::utility::is_approximately_equal(distance,
                                             Config::lookahead_distance)) {
      return std::atan2(optimal_path->path_x.at(iter),
                        optimal_path->path_y.at(iter));
    } else if (distance > Config::lookahead_distance) {
      // find a point lookahead_distance away
      opt_assert(optimal_path->path_x.size());
      const auto x_prev = optimal_path->path_x[iter - 1];
      const auto y_prev = optimal_path->path_y[iter - 1];
      const auto distance_prev = std::hypot(x_prev, y_prev);
      const auto distance_diff = distance - distance_prev;
      const auto x_diff = optimal_path->path_x.at(iter) - x_prev;
      const auto y_diff = optimal_path->path_y.at(iter) - y_prev;
      const auto x_lookahead =
          x_prev +
          (Config::lookahead_distance - distance_prev) / distance_diff * x_diff;
      const auto y_lookahead =
          y_prev +
          (Config::lookahead_distance - distance_prev) / distance_diff * y_diff;
      return std::atan2(x_lookahead, y_lookahead);
    }
  }

  return 0.;
}

constexpr double
steering_angle_to_fraction(double steering_angle_rad) noexcept {
  const auto fraction =
      steering_angle_rad / opt::utility::deg_to_rad(Config::max_steering_angle);
  return std::clamp(fraction, -1.0, 1.0);
}
} // namespace Control
