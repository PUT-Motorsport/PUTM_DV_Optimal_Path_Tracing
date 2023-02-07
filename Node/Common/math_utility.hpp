#pragma once

#include <cmath>

namespace opt::utility {

template <typename T>
constexpr bool is_approximately_equal(T value, T value2,
                                      T epsilon = 10e-10) noexcept {
  static_assert(std::is_floating_point_v<T>);
  return (std::abs(value) - std::abs(value2)) < epsilon;
}

template <typename T>
constexpr bool is_in_range(T range_bound_a, T range_bound_b, T value) noexcept {
  static_assert(std::is_arithmetic_v<T>);
  return (value > range_bound_a and value < range_bound_b) or
         (value < range_bound_a and value > range_bound_b);
}

template <typename T> constexpr T deg_to_rad(T degrees) noexcept {
  static_assert(std::is_floating_point_v<T>);
  return degrees / 180. * M_PI;
}

template <typename T> constexpr T rad_to_deg(T radians) noexcept {
  static_assert(std::is_floating_point_v<T>);
  return radians / M_PI * 180.;
}
} // namespace opt::utility
