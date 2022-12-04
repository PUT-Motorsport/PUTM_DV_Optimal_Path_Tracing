#pragma once

#include <type_traits>
#include <span>
#include <vector>
#include <cmath>
#include "debug_utility.hpp"

namespace opt::spline {

    template<typename T>
    requires std::is_floating_point<T>::value
    struct SplineFragment {
        T x;
        T a;
        T b;
        T c;
        T d;
    };

    template<typename T>
    requires std::is_floating_point_v<T>
    class NaturalSpline {
    public:
        explicit constexpr NaturalSpline(std::span<T> t, std::span<T> y);

        [[nodiscard]] constexpr
        T get_at(T arg) const noexcept;

        [[nodiscard]] constexpr
        std::vector<T> get_range(std::span<T> arg_range) const;

        [[nodiscard]] constexpr
        T get_derivative_at() const noexcept;

        [[nodiscard]] constexpr
        T get_second_derivative_at() const noexcept;

        std::vector<SplineFragment<T>> get_spline_fragments() const {
            return spline_fragments;
        }

        std::size_t size() {
            return spline_fragments.size();
        }

    private:
        std::vector<SplineFragment<T>> spline_fragments;
    };

    template<typename T>
    requires std::is_floating_point_v<T> constexpr
    NaturalSpline<T>::NaturalSpline(std::span<T> t, std::span<T> y) {
        //https://en.wikipedia.org/w/index.php?title=Spline_%28mathematics%29&oldid=288288033#Algorithm_for_computing_natural_cubic_splines
        opt_assert(t.size() == y.size());
        const auto n = t.size() - 1;
        spline_fragments.reserve(n + 1);

        for (std::size_t index = 0; index < y.size(); ++index) {
            spline_fragments[index].a = y[index];
        }

        std::vector<T> h{n};
        for (std::size_t index = 0; index < n; ++index) {
            h[index] = spline_fragments[index + 1].x - spline_fragments[index].x;
        }
        std::vector<T> alpha{n};
        for (std::size_t index = 1; index < n; ++index) {
            alpha[index] = 3 / h[index] * (spline_fragments.a[index + 1] - spline_fragments.a[index]) -
                    3 / h[index - 1] * (spline_fragments.a[index] - spline_fragments.a[index - 1]);
        }
        std::vector<T> l{n + 1};
        std::vector<T> mu{n + 1};
        std::vector<T> z{n + 1};
        l[0] = 1;
        mu[0] = 0;
        z[0] = 0;
        for (std::size_t index = 1; index < n; ++index) {
            l[index] = 2 * (spline_fragments[index + 1].x - spline_fragments[index - 1]) - h[index - 1] * mu[index - 1];
            mu[index] = h[index] / l[index];
            z[index] = (alpha[index] - h[index - 1] * z[index - 1]) / l[index];
        }
        l[n] = 1;
        z[n] = 0;
        spline_fragments[n].c = 0;

        for (int32_t index = n - 1; index >= 0; --index) {
            spline_fragments[index].c = z[index] - mu[index] * spline_fragments[index + 1].c;
            spline_fragments[index].b = (spline_fragments[index + 1].a - spline_fragments[index].a) / h[index] -
                    h[index] / 3 * (spline_fragments[index + 1].c + 2 * spline_fragments[index].c);
            spline_fragments[index].d = (spline_fragments[index + 1].c - spline_fragments[index].c) / (3 * h[index]);
        }
    }

    template<typename T>
    requires std::is_floating_point_v<T>constexpr
    T NaturalSpline<T>::get_at(T arg) const noexcept {
        std::size_t index = 0;
        while (arg < spline_fragments[index].x) {
            ++index;
            opt_assert(index < spline_fragments.size());
        }
        const T delta_x{arg - spline_fragments[index].x};

        auto& fragment = spline_fragments[index];

        return fragment.a + fragment.b * delta_x + fragment.c * std::pow(delta_x, 2) + fragment.d * std::pow(delta_x, 2);
    }

}   // namespace Spline