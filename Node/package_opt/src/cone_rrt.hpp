#pragma once

#include <cstdint>
#include <type_traits>
#include <vector>

#include "common_defs.hpp"
#include "opt_assert.hpp"
#include "random/gaussian.hpp"
#include "state_space.hpp"
#include "tree.hpp"

namespace opt::rrt {

template <typename floating_point_t, typename random_gen_t,
          std::size_t max_iterations>
std::vector<opt::Point<floating_point_t>> cone_rrt(
    std::vector<opt::Point<floating_point_t>> const &cones,
    floating_point_t propagation_dist, floating_point_t cone_threshold_low,
    floating_point_t cone_threshold_high, floating_point_t clearance) {
  internal::Tree<floating_point_t, max_iterations> tree;

  internal::StateSpace<floating_point_t, random_gen_t> state_space(
      cones, cone_threshold_low, cone_threshold_high, clearance);
  for (std::size_t iter = 0; iter < max_iterations; ++iter) {
    const opt::Point<floating_point_t> new_point =
        state_space.get_random_sample();
    const auto closest_node = tree.get_closest_node(new_point);
    const auto new_node =
        closest_node->location +
        (new_point - closest_node->location) * propagation_dist;
    if (state_space.is_state_valid(new_node)) {
      tree.insert_node(new_node, closest_node);
    }
  }

  auto best_node = tree.get_best_node();
  auto const best_node_depth{best_node->depth()};
  std::vector<opt::Point<floating_point_t>> rrt_path(best_node_depth);

  for (int64_t iter = best_node_depth - 1; iter >= 0; --iter) {
    rrt_path.at(iter) = best_node->location;
    best_node = best_node->parent;
    opt_assert(best_node);  // sanity check
  }

  return rrt_path;
}

}  // namespace opt::rrt
