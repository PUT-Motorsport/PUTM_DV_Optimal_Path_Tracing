#pragma once

#include <cmath>

#include "../../package_opt/src/common_defs.hpp"

// bicycle kinematic model for control algorithms testing
// derivation: https://youtu.be/HqNdBiej23I

class BicycleModel {
    using PointD = opt::Point<double>;
    using Vector2D = opt::Point<double>;
public:
    constexpr BicycleModel(PointD center_of_mass_location, double heading, double distance_rear, double distance_front, double delta_time) noexcept :
            heading(heading), center_of_mass_location(center_of_mass_location), distance_rear(distance_rear), distance_front(distance_front),
            delta_time(delta_time) {

    }

    opt::Point<double> update(double velocity, double steering_angle) noexcept {
        const auto dists = distance_rear / (distance_front + distance_rear);
        const auto sideslip_angle = std::atan(dists * std::tan(steering_angle));
        center_of_mass_location.x += velocity * std::cos(heading + sideslip_angle) * delta_time;
        center_of_mass_location.y += velocity * std::sin(heading + sideslip_angle) * delta_time;

        heading += velocity / (distance_rear + distance_front) * std::cos(sideslip_angle) *
                std::tan(steering_angle) * delta_time;

        return center_of_mass_location;
    }

    [[nodiscard]] opt::Point<double> get_center_of_mass_location() const noexcept {
        return center_of_mass_location;
    }

    [[nodiscard]] opt::Point<double> get_front_axle_location() const noexcept {
       const Vector2D front_axle_direction = Vector2D{std::cos(heading), std::sin(heading)}.normalized();

       return center_of_mass_location + front_axle_direction * distance_front;
    }

    [[nodiscard]] opt::Point<double> get_rear_axle_location() const noexcept {
        const Vector2D rear_axle_direction = Vector2D{std::cos(heading), std::sin(heading)}.normalized();

        return center_of_mass_location + rear_axle_direction * -distance_front;
    }

    [[nodiscard]] double get_heading() const noexcept {
        return heading;
    }

private:
    double heading;
    PointD center_of_mass_location;
    const double distance_rear;
    const double distance_front;
    const double delta_time;
};