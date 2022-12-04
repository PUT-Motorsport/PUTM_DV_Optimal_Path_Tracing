#include "callbacks.h"

#include "distance.hpp"
#include "debug_utility.hpp"
#include "spline.hpp"
#include <span>

namespace opt {

    void on_cones_received_callback(package_opt::Cones::ConstPtr const &cones) noexcept {
        //@brief This is the main function, called on receipt of new cones positions
        opt_assert(cones.cones_left_x.size() == cones.cones_left_y.size());
        opt_assert(cones.cones_right_x.size() == cones.cones_right_y.size());
        //Parametrize the splines by L2 distance between the cones
        std::vector<double> left_parameter(cones.cones_left_x.size());
        left_parameter.at(0) = 0;
        for (std::size_t iter = 1; iter < left_parameter.size(); ++iter) {
            left_parameter.at(iter) = opt::utility::L2_distance(cones.cones_left_x.at(iter - 1),
                                                                cones.cones_left_x.at(iter),
                                                  cones.cones_left_y.at(iter - 1), cones.cones_left_y.at(iter));
        }
        std::vector<double> right_parameter(cones.cones_right_x.size());
        right_parameter.at(0) = 0;
        for(std::size_t iter = 1; iter < right_parameter.size(); ++iter) {
            right_parameter.at(iter) = opt::utility::L2_distance(cones.cones_right_x.at(iter - 1),
                                                                 cones.cones_right_x.at(iter),
                                                                 cones.cones_left_x.at(iter - 1),
                                                                 cones.cones_left_y.at(iter));
        }

        //Find left_x, left_y, right_x, right_y splines

        opt::spline::NaturalSpline<double> right_x_spline(right_parameter, std::span<double>{cones.cones_right_x.begin(), cones.cones_right_x.end()});
//        opt::spline::NaturalSpline<double> right_y_spline(right_parameter, std::span{cones.cones_right_y});
//        opt::spline::NaturalSpline<double> left_x_spline(left_parameter, std::span{cones.cones_left_x});
        opt::spline::NaturalSpline<double> left_y_spline(left_parameter, std::span<double>{cones.cones_left_y.data(), cones.cones_left_y.size()});
        //Extrapolate to a higher number of points

        //Use the extrapolated data to find the centerline

        //todo: do minimal curvature optimization, for now: pass the generated centerline to the controller
    }
} //namespace opt