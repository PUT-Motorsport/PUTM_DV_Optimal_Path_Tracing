/*
 * Testing the Stanley algorithm outside ROS
 * */

#include <vector>

#include "bicycle_model.hpp"
#include "../math_utility.hpp"
#include "../csv.h"

int main() {
    BicycleModel model(opt::Point<double>{0.0, 0.0}, 0.0, 1.0, 1.0, 0.01);

    constexpr std::size_t iteration_count{1000};
    constexpr auto velocity{5.0};
    constexpr auto steering_angle{opt::utility::deg_to_rad(15.0)};

    std::vector<opt::Point<double>> points;
    points.reserve(iteration_count);

    for (std::size_t _ = 0; _ < iteration_count; ++_) {
        const auto location = model.update(velocity, steering_angle);
        points.emplace_back(location);
    }
    if (csv::dump("path.csv", points)) {
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}