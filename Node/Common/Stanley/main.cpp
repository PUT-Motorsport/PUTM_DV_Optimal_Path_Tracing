#include "stanley.h"
#include "../math_utility.hpp"
#include "../Bicycle/bicycle_model.hpp"
#include "../csv.h"

#include <iostream>

int main() {
    const auto path = csv::load2Dcoords("track.csv");
    if (path.empty()) {
        std::cout << "Path not loaded" << std::endl;
        std::terminate();
    }
    // controller parameters
    constexpr auto max_steering_angle = opt::utility::deg_to_rad(45.);
    constexpr auto cross_track_error_gain{1.0};
    constexpr auto lookahead{1.0};
    StanleyController<double> stanley(path, max_steering_angle, cross_track_error_gain, lookahead);

    // model parameters
    const opt::Point<double> start = path.at(0);
    constexpr auto heading{0.0};
    constexpr auto l_r{1.0};
    constexpr auto l_f{1.0};
    // simulation parameters
    constexpr std::size_t iteration_count{50000};
    constexpr auto velocity{3.0};
    constexpr auto delta_time{0.1};

    std::vector<opt::Point<double>> locations;
    locations.reserve(iteration_count);

    BicycleModel model(start, heading, l_r, l_f, delta_time);
    for (std::size_t _ = 0; _ < iteration_count; ++_) {
        const auto pos = model.get_front_axle_location();
        const auto heading = model.get_heading();
        const auto steering_angle = stanley.get_angle(pos, heading, velocity);
        const auto center_location = model.update(velocity, steering_angle);
        locations.emplace_back(center_location);
    }
    csv::dump("stanley.csv", locations);
}