#pragma once

#include <cstdint>

namespace opt::config {

    //todo: set all parameters
constexpr auto cone_radius{0.8};
constexpr auto max_calculation_time{0.2};
constexpr std::size_t max_cones_horizon{6};
constexpr auto track_discretization_interval{0.1};
constexpr auto track_optimality_threshold{9.0};
}
