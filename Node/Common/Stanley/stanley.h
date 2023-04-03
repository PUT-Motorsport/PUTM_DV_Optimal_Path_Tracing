#pragma once

#include "../../package_opt/src/common_defs.hpp"
#include "../splines.hpp"

#include <algorithm>
#include <limits>
#include <cmath>
#include <vector>

template<typename T>
class StanleyController {
public:
    StanleyController(std::vector<opt::Point<T>> const &path, const T max_steering_angle,
                      const T cross_track_error_gain, const T lookahead, const T low_speed_softening = 0.0) noexcept :
    path(path), low_speed_softening(low_speed_softening), cross_track_error_gain(cross_track_error_gain), max_steering_angle(max_steering_angle),
    lookahead(lookahead) {
        static_assert(std::is_floating_point_v < T > );
    }

    T get_angle(opt::Point <T> current_pos, T heading, T velocity) noexcept {
        // cross track error

        // find the closest path point
        std::size_t closest_index{};
        T min_distance{std::numeric_limits<T>::max()};
        for (std::size_t iter = 0; iter < path.size(); ++iter) {
            if (path.at(iter).distance(current_pos) < min_distance) {
                closest_index = iter;
                min_distance = path.at(iter).distance(current_pos);
            }
        }
        const auto cross_track_error = (current_pos - path.at(closest_index)).distance();

        T correct_heading;
        // find the lookahead point
        for (std::size_t iter = closest_index; iter < path.size(); ++iter) {
           if (current_pos.distance(path.at(iter)) > lookahead) {
               // fixme: lookahead not constant
               const auto current_vehicle_heading_vector = opt::Point<T>{std::cos(heading), std::sin(heading)};
               const auto vehicle_correct_heading_vector = path.at(iter) - current_pos;
               correct_heading = std::acos(current_vehicle_heading_vector.dot(vehicle_correct_heading_vector)
                       / (current_vehicle_heading_vector.length() * vehicle_correct_heading_vector.length()));
               break;
           }
        }

        const auto angle = correct_heading + std::atan(cross_track_error_gain * cross_track_error / (velocity + low_speed_softening) );

        return std::clamp(angle, -max_steering_angle, max_steering_angle);
    }

private:
    const std::vector <opt::Point<T>> path;
    const T low_speed_softening; // apply a constant to soften the controller at low speeds
    const T cross_track_error_gain;
    const T max_steering_angle;
    const T lookahead;
};