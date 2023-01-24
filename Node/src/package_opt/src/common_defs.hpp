#pragma once

#include <type_traits>

namespace opt {

template <typename T> struct Point {
  T x;
  T y;
  static_assert(std::is_floating_point_v<T>);
};

template <typename T> struct Edge {
  Point<T> a;
  Point<T> b;
};

} // namespace opt
