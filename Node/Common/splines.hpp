#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>
#include <vector>
#include <cassert>

#include "opt_assert.hpp"

namespace opt::spline {

template<typename T>
class CubicSpline {
public:
    constexpr CubicSpline(std::vector<T> const &x, std::vector<T> const &y) noexcept;

    [[nodiscard]] constexpr T get_at(T arg) noexcept;
    [[nodiscard]] constexpr std::vector<T> get_range(T start, T stop, T increment) noexcept;

private:
    std::vector<T> x, a, b, c, d;

    [[nodiscard]] constexpr std::size_t get_index(T arg) noexcept;
};

template<typename T>
constexpr std::vector<T> CubicSpline<T>::get_range(T start, T stop, T increment) noexcept {
    auto index = get_index(start);
    const std::size_t space_required = std::floor((stop - start) / increment);
    std::vector<T> result;
    result.reserve(space_required);
    while (start < stop) {
        const auto diff_x = start - x.at(index);
        assert(diff_x >= 0.0);
        result.emplace_back(a.at(index) + b.at(index) * diff_x + c.at(index) * std::pow(diff_x, 2) + d.at(index) * std::pow(diff_x, 3));
        start += increment;
        if (start > x.at(index + 1)) {
           index++; // fixme: out of bounds?
        }
    }
    return result;
}

template<typename T>
constexpr std::size_t CubicSpline<T>::get_index(const T arg) noexcept {
    assert(arg >= x.at(0));
    assert(arg <= x.back());
    std::size_t index{};
    while (x.at(index) < arg) {
        index++;
    }
    return index;
}


template<typename T>
constexpr
CubicSpline<T>::CubicSpline(const std::vector<T> &x, const std::vector<T> &y) noexcept : x(x), a(y), b(y.size() - 1),
                                                                                         c(y.size()),
                                                                                         d(y.size() - 1) {
    const std::size_t n = x.size() - 1;
    assert(n >= 0);
    assert(x.size() == y.size());

    std::vector<double> h(n);
    std::vector<double> alpha(n);

    for (std::size_t iter = 0; iter < n; ++iter) {
        h.at(iter) = x.at(iter + 1) - x.at(iter);
    }
    alpha.at(0) = 0;
    for (std::size_t iter = 1; iter < alpha.size(); ++iter) {
        alpha.at(iter) = 3 * (a.at(iter + 1) - a.at(iter)) / h.at(iter) -
                3 / h.at(iter - 1) * (a.at(iter) - a.at(iter - 1));
    }
    std::vector<double> l(n + 1);
    std::vector<double> mu(n + 1);
    std::vector<double> z(n + 1);
    l.at(0) = 1;
    mu.at(0) = 0;
    z.at(0) = 0;

    for (std::size_t iter = 1; iter < n; ++iter) {
        l.at(iter) = 2 * (x.at(iter + 1) - x.at(iter - 1)) - h.at(iter - 1) * mu.at(iter - 1);
        mu.at(iter) = h.at(iter) / l.at(iter);

        z.at(iter) = (alpha.at(iter) - h.at(iter - 1) * z.at(iter - 1)) / l.at(iter);
    }
    l.at(n) = 1;
    z.at(n) = 0;
    c.at(n) = 0;

    for (int64_t iter = n - 1; iter >= 0; --iter) {
        c.at(iter) = z.at(iter) - mu.at(iter) * c.at(iter + 1);
        b.at(iter) = (a.at(iter + 1) - a.at(iter)) / h.at(iter) - h.at(iter) * (c.at(iter + 1) + 2 * c.at(iter)) / 3;
        d.at(iter) = (c.at(iter + 1) - c.at(iter)) / (3 * h.at(iter));
    }
}

template<typename T>
[[nodiscard]] constexpr T CubicSpline<T>::get_at(const T arg) noexcept {
    const auto index = get_index(arg);
    const auto diff_x = arg - x.at(index);
    assert(diff_x >= 0);
    return a.at(index) + b.at(index) * arg + c.at(index) * std::pow(arg, 2) + d.at(index) * std::pow(arg, 3);
}

}  // namespace opt::spline
