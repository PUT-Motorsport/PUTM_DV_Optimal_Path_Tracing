#pragma once

#include <stdio.h>

#include <optional>
#include <utility>

#include "Delaunay/edge.h"
#include "common_defs.hpp"

namespace opt::utility {

template <typename T>
constexpr bool is_in_range(T range_bound_a, T range_bound_b, T value) {
  static_assert(std::is_arithmetic_v<T>);
  return (value > range_bound_a and value < range_bound_b) or
         (value < range_bound_a and value > range_bound_b);
}
template <typename T>
constexpr bool find_intersection_point(dt::Edge<T> const &first_line,
                                       Edge<T> const &second_line) noexcept {
  // this function takes instances of objects defined in libraries

  // find first and second line equations
  const T a1 =
      (first_line.w->y - first_line.v->y) / (first_line.w->x - first_line.v->x);
  const T a2 =
      (second_line.b.y - second_line.a.y) / (second_line.b.x - second_line.a.x);

  const T b1 = first_line.v->y - a1 * first_line.v->x;
  const T b2 = second_line.a.y - a2 * second_line.a.x;
  const T intersection_x = (b2 - b1) / (a1 - a2);
  if (is_in_range(second_line.a.x, second_line.b.x, intersection_x) and
      is_in_range(first_line.v->x, first_line.w->x, intersection_x)) {
    return true;
  }
  return false;
}

}  // namespace opt::utility
