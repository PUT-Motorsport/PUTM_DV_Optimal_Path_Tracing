#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "opt_assert.hpp"
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

namespace opt::spline {

template <typename T> struct SplineFragment {
  T x;
  T a;
  T b;
  T c;
  T d;
  static_assert(std::is_floating_point_v<T>);
};

template <typename T> class NaturalSpline {
public:
  explicit /*constexpr*/ NaturalSpline(std::vector<T> const &t,
                                       std::vector<T> const &y);

  [[nodiscard]] constexpr T get_at(T arg) const noexcept;

  [[nodiscard]] constexpr std::vector<T> get_range(T start, T stop,
                                                   T increment) const;

  [[nodiscard]] constexpr std::vector<T>
  get_n_points(std::size_t points_count) const;

  [[nodiscard]] constexpr T get_derivative_at(T arg) const noexcept;

  [[nodiscard]] constexpr T get_second_derivative_at(T arg) const noexcept;

  const std::vector<SplineFragment<T>> &get_spline_fragments() const {
    return spline_fragments;
  }

  [[nodiscard]] constexpr std::size_t size() const noexcept {
    return spline_fragments.size();
  }

private:
  std::vector<SplineFragment<T>> spline_fragments;
  std::size_t get_index(T arg) const noexcept;
};

template <typename T>
// constexpr
NaturalSpline<T>::NaturalSpline(std::vector<T> const &t,
                                std::vector<T> const &y)
    : spline_fragments(t.size() + 1) {
  // https://en.wikipedia.org/w/index.php?title=Spline_%28mathematics%29&oldid=288288033#Algorithm_for_computing_natural_cubic_splines
  static_assert(std::is_floating_point_v<T>);
  opt_assert(t.size() == y.size()) const auto n = t.size() - 1;
  // spline_fragments.reserve(n + 1);

  for (std::size_t index = 0; index < y.size(); ++index) {
    spline_fragments[index].a = y[index];
  }
  std::vector<T> h(n);
  for (std::size_t index = 0; index < n; ++index) {
    h[index] = spline_fragments[index + 1].x - spline_fragments[index].x;
  }
  std::vector<T> alpha(n);
  for (std::size_t index = 1; index < n; ++index) {
    alpha[index] =
        3 / h[index] *
            (spline_fragments.at(index + 1).a - spline_fragments[index].a) -
        3 / h[index - 1] *
            (spline_fragments.at(index).a - spline_fragments.at(index - 1).a);
  }

  std::vector<T> l(n + 1);
  std::vector<T> mu(n + 1);
  std::vector<T> z(n + 1);
  l[0] = 1;
  mu[0] = 0;
  z[0] = 0;
  for (std::size_t index = 1; index < n; ++index) {
    l[index] =
        2 * (spline_fragments[index + 1].x - spline_fragments[index - 1].x) -
        h[index - 1] * mu[index - 1];
    mu[index] = h[index] / l[index];
    z[index] = (alpha[index] - h[index - 1] * z[index - 1]) / l[index];
  }
  l[n] = 1;
  z[n] = 0;
  spline_fragments[n].c = 0;

  for (int32_t index = n - 1; index >= 0; --index) {
    spline_fragments[index].c =
        z[index] - mu[index] * spline_fragments[index + 1].c;
    spline_fragments[index].b =
        (spline_fragments[index + 1].a - spline_fragments[index].a) / h[index] -
        h[index] / 3 *
            (spline_fragments[index + 1].c + 2 * spline_fragments[index].c);
    spline_fragments[index].d =
        (spline_fragments[index + 1].c - spline_fragments[index].c) /
        (3 * h[index]);
  }
}

template <typename T>
constexpr T NaturalSpline<T>::get_at(T arg) const noexcept {
  const auto index = get_index(arg);
  const T delta_x{arg - spline_fragments[index].x};

  auto &fragment = spline_fragments[index];

  return fragment.a + fragment.b * delta_x + fragment.c * std::pow(delta_x, 2) +
         fragment.d * std::pow(delta_x, 2);
}

template <typename T>
constexpr std::vector<T> NaturalSpline<T>::get_range(T start, T stop,
                                                     T increment) const {
  // the purpose of this function is to optimize generating a range of values by
  // minimizing the amount of searches needed start is inclusive, stop is
  // exclusive
  opt_assert(increment > 0);
  opt_assert(start < stop);
  // find starting x: biggest x to satisfy x < arg

  const std::size_t required_vector_size{
      std::ceil((stop - start) / increment)}; // fixme: floor or ceil??
  std::vector<T> y_range;
  y_range.reserve(required_vector_size);
  auto spline_index = get_index(start);
  while (start < stop) {
    const T delta_arg{start - spline_fragments[spline_index].x};
    opt_assert(delta_arg <= 0);

    y_range.emplace_back(
        spline_fragments[spline_index].d * std::pow(delta_arg, 3) +
        spline_fragments[spline_index].c * std::pow(delta_arg, 2) +
        spline_fragments[spline_index].b + delta_arg +
        spline_fragments[spline_index].a);
    start += increment;
    // check if the next spline fragment is now the biggest that start <
    // spline_fragments
    if (start > spline_fragments[spline_index + 1].x) {
      ++spline_index;
    }
  }
  return y_range;
}

template <typename T>
constexpr std::vector<T>
NaturalSpline<T>::get_n_points(std::size_t points_count) const {
  // generate points_count points in the whole spline
  const auto start = spline_fragments[0].x;
  const auto stop = spline_fragments.back().x;
  const auto increment = (stop - start) / points_count;
  return this->get_range(start, stop, increment);
}

template <typename T>
constexpr T NaturalSpline<T>::get_derivative_at(T arg) const noexcept {
  // calculate the derivative at point arg using spline coefficients
  const auto index = get_index(arg);
  const T delta_x{arg - spline_fragments[index].x};

  constexpr auto &fragment = spline_fragments[index];
  return 3 * fragment.d * std::pow(delta_x, 2) + 2 * fragment.c * delta_x +
         fragment.b;
}

template <typename T>
constexpr T NaturalSpline<T>::get_second_derivative_at(T arg) const noexcept {
  // calculate the second derivative at point arg using spline coefficients
  const auto index = get_index(arg);
  const T delta_x{arg - spline_fragments[index].x};

  constexpr auto &fragment = spline_fragments[index];
  return 6 * fragment.d * delta_x + 2 * fragment.c;
}

template <typename T>
std::size_t NaturalSpline<T>::get_index(T arg) const noexcept {
  opt_assert(arg >= spline_fragments[0].x);
  if (arg < spline_fragments[0].x) {
    ROS_INFO("Assert. x: %f arg: %f", spline_fragments[0].x, arg);
  }
  opt_assert(arg <= spline_fragments.back().x);
  std::size_t index = 0;
  while (arg < spline_fragments[index].x) {
    ++index;
    opt_assert(index < spline_fragments.size());
  }
  return index;
}

} // namespace opt::spline
