#include "spline_interpolation.hpp"

#include "splines.hpp"
#include "config.hpp"
#include "opt_assert.hpp"

#include <utility>
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

namespace opt
{
    std::pair < std::vector<double>, std::vector<double>> track_spline_interpolation(std::vector<double> const &x, std::vector<double> const &y)
    {
        opt_assert(x.size() == y.size());
        std::vector<double> t(x.size());
        t[0] = 0;

        for (std::size_t i = 1; i < x.size(); ++i)
        {
            t[i] = t[i - 1] + std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
            ROS_INFO("%f", t[i]);
        }

        opt::spline::NaturalSpline<double> spline_x(t, x);
        opt::spline::NaturalSpline<double> spline_y(t, y);

        return std::pair{spline_x.get_range(t.front(), t.back(), opt::config::track_discretization_interval),
                         spline_y.get_range(t.front(), t.back(), opt::config::track_discretization_interval)};
    }

} // namespace opt