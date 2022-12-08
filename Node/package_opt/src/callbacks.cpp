#include "callbacks.h"

#include "distance.hpp"
#include "debug_utility.hpp"
#include "spline.hpp"
#include "config.hpp"
#include <span>
#include <numeric>

extern ros::Publisher optimal_path_publisher;

namespace opt {

    void on_cones_received_callback(package_opt::Cones::ConstPtr const &cones) {
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

        const opt::spline::NaturalSpline<double> right_x_spline(right_parameter, std::span{cones.cones_right_x});
        const opt::spline::NaturalSpline<double> right_y_spline(right_parameter, std::span{cones.cones_right_y});
        const opt::spline::NaturalSpline<double> left_x_spline(left_parameter, std::span{cones.cones_left_x});
        const opt::spline::NaturalSpline<double> left_y_spline(left_parameter, std::span{cones.cones_left_y});

        //Extrapolate to a higher number of points
        const auto approximate_track_length = ((right_parameter.back() > left_parameter.back()) ? right_parameter.back() : left_parameter.back());
        const std::size_t number_of_discretization_points = std::ceil(approximate_track_length / opt::config::track_discretization_interval);

        const auto track_right_x =  right_x_spline.get_n_points(number_of_discretization_points);
        const auto track_right_y = right_x_spline.get_n_points(number_of_discretization_points);
        const auto track_left_x = left_x_spline.get_n_points(number_of_discretization_points);
        const auto track_left_y = left_y_spline.get_n_points(number_of_discretization_points);

        //Use the extrapolated data to find the centerline
        std::vector<double> centerline_x(number_of_discretization_points);
        std::vector<double> centerline_y(number_of_discretization_points);

        for (std::size_t left_side_iter = 0; left_side_iter < number_of_discretization_points; ++left_side_iter) {
                double minimal_distance_so_far = std::numeric_limits<double>::max();
                std::size_t minimal_distance_point_index{};
                for (std::size_t right_side_iter = std::max<int64_t>(0, static_cast<int64_t>(left_side_iter) - static_cast<int64_t>(opt::config::track_bounds_search_narrowing_heuristic));
                        right_side_iter < std::min(left_side_iter, left_side_iter + opt::config::track_bounds_search_narrowing_heuristic); ++right_side_iter) {
                                const auto distance = opt::utility::L2_distance(track_right_x[right_side_iter],
                                                                                 track_left_x[left_side_iter],
                                                                                 track_right_y[right_side_iter],
                                                                                 track_left_y[left_side_iter]);
                                if (minimal_distance_so_far > distance) {
                                        minimal_distance_so_far = distance;
                                        minimal_distance_point_index = right_side_iter;
                                }
                        }
                centerline_x[left_side_iter] = (track_left_x[left_side_iter] + track_right_x[minimal_distance_point_index]) / 2;
                centerline_y[left_side_iter] = (track_left_y[left_side_iter] + track_right_y[minimal_distance_point_index]) / 2;
        }

        // todo: do minimal curvature optimization, for now: pass the generated centerline to the controller
        //Create the ros message
        package_opt::optimal_path ros_msg{std::move(centerline_x), std::move(centerline_y)};

        optimal_path_publisher.publish(ros_msg);
    }
} //namespace opt