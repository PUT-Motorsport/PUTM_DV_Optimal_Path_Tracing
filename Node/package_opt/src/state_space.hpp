#pragma once

#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include "common_defs.hpp"

namespace opt::internal {

template <typename floating_point_t, typename random_gen_t>
class StateSpace {
 public:
  constexpr StateSpace(
      std::vector<opt::Point<floating_point_t>> const &obstacles,
      floating_point_t sampling_region_bound_low,
      floating_point_t sampling_region_bound_high, floating_point_t clearance)
      : obstacles(obstacles), clearance(clearance) {
    // determine the state space properties for sampling functions
    opt::Point<floating_point_t> center;
    std::size_t count{};
    floating_point_t std_dev_x, std_dev_y;

    do {
      center.x = 0;
      center.y = 0;
      count = 0;
      std_dev_x = 0.;
      std_dev_y = 0.;
      for (auto const &obstacle : obstacles) {
        if (obstacle.distance() < sampling_region_bound_high and
            obstacle.distance() > sampling_region_bound_low) {
          count++;
          center += obstacle;
          if (std_dev_x < obstacle.x) {
            std_dev_x = obstacle.x;
          }
          if (std_dev_y < obstacle.y) {
            std_dev_y = obstacle.y;
          }
        }
      }
      sampling_region_bound_low *= 0.9;
      sampling_region_bound_high *= 0.9;
    } while (count == 0);
    center /= static_cast<floating_point_t>(count);
    sampling_x = random_gen_t(center.x, std_dev_x);
    sampling_y = random_gen_t(center.y, std_dev_y);
  }

  constexpr opt::Point<floating_point_t> get_random_sample() noexcept {
    return opt::Point<floating_point_t>{sampling_x.get(), sampling_y.get()};
  }

  constexpr bool is_state_valid(
      opt::Point<floating_point_t> const &point) const noexcept {
    for (auto const &obstacle : obstacles) {
      if (point.distance(obstacle) < clearance) {
        return false;
      }
    }
    return true;
  }

 private:
  std::vector<opt::Point<floating_point_t>> obstacles;
  random_gen_t sampling_x;
  random_gen_t sampling_y;
  floating_point_t const clearance;
};
}  // namespace opt::internal
