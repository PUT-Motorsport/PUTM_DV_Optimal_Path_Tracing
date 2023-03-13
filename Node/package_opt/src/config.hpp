#pragma once

#include <cstdint>

namespace opt::config {

// todo: set all parameters
constexpr auto cone_radius{0.5};
constexpr auto max_calculation_time{0.2};
constexpr auto track_optimality_threshold{9.0};
constexpr auto cone_threshold_low{5.};
constexpr auto cone_threshold_high{8.};

// cone rrt
constexpr std::size_t max_iterations{200};
constexpr auto propagation_dist{0.1};
}  // namespace opt::config
