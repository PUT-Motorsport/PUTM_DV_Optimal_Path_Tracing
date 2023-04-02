#pragma once

#include <cmath>
#include <type_traits>

#include "function_attributes.hpp"

namespace opt {

template <typename floating_point_t>
struct Point {
  floating_point_t x;
  floating_point_t y;
  static_assert(std::is_floating_point_v<floating_point_t>);

  constexpr floating_point_t distance() const noexcept pure_function {
    return std::hypot(x, y);
  }

  constexpr floating_point_t distance(
      Point<floating_point_t> const &b) const noexcept pure_function {
    return std::hypot(x - b.x, y - b.y);
  }

  constexpr Point<floating_point_t> operator+(
      Point<floating_point_t> const &b) const noexcept pure_function {
    return Point<floating_point_t>{x + b.x, y + b.y};
  }

  constexpr Point<floating_point_t> operator-(
      Point<floating_point_t> const &b) const noexcept pure_function {
    return Point<floating_point_t>{x - b.x, y - b.y};
  }

  constexpr Point<floating_point_t> operator*(
      floating_point_t scalar) const noexcept pure_function {
    return Point<floating_point_t>{x * scalar, y * scalar};
  }

  constexpr Point<floating_point_t> operator+=(
      Point<floating_point_t> const &b) noexcept pure_function {
    x += b.x;
    y += b.y;
    return *this;
  }

  constexpr Point<floating_point_t> operator-=(
      Point<floating_point_t> const &b) noexcept pure_function {
    x -= b.x;
    y -= b.y;
    return *this;
  }

  constexpr Point<floating_point_t> operator*=(floating_point_t scalar) noexcept
      pure_function {
    x *= scalar;
    y *= scalar;
    return *this;
  }

  constexpr Point<floating_point_t> operator/=(floating_point_t scalar) noexcept
      pure_function {
    x /= scalar;
    y /= scalar;
    return *this;
  }

  constexpr floating_point_t length() noexcept pure_function {
     return std::hypot(x, y);
  }

  constexpr Point<floating_point_t> normalized() noexcept pure_function {
      return *this / this->length;
  }
};

template <typename floating_point_t>
struct Edge {
  Point<floating_point_t> a;
  Point<floating_point_t> b;
};

}  // namespace opt
