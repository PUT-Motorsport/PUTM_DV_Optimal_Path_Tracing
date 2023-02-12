#pragma once

#include <cstdint>
#include <limits>
#include <type_traits>

#include "common_defs.hpp"
#include "function_attributes.hpp"

namespace opt::internal {

template <typename floating_point_t>
struct Node {
  static_assert(std::is_floating_point_v<floating_point_t>);
  opt::Point<floating_point_t> location;
  Node<floating_point_t> const *parent;

  constexpr std::size_t depth() const noexcept pure_function {
    std::size_t depth{0};
    Node<floating_point_t> const *current_node = this->parent;
    while (current_node) {
      depth++;
      current_node = current_node->parent;
    }
    return depth;
  }
};

template <typename floating_point_t, std::size_t max_iterations>
class Tree {
 public:
  constexpr Tree() noexcept
      : current_tree_size(1) {  // node_array should not be initialized now
    static_assert(std::is_trivially_constructible<Node<floating_point_t>>(),
                  "Node type should be trivially constructible");
    static_assert(std::is_floating_point_v<floating_point_t>);
    node_array[0].location = opt::Point<floating_point_t>{0.0, 0.0};
    node_array[0].parent = nullptr;
  }
  virtual ~Tree() {}

  constexpr Node<floating_point_t> const *get_closest_node(
      opt::Point<floating_point_t> const &point) const noexcept {
    std::size_t closest_node_index{};
    floating_point_t closest_node_distance =
        std::numeric_limits<floating_point_t>::max();
    for (std::size_t iter = 0; iter < current_tree_size; ++iter) {
      if (point.distance(node_array[iter].location) < closest_node_distance) {
        closest_node_distance = point.distance(node_array[iter].location);
        closest_node_index = iter;
      }
    }
    return node_array + closest_node_index;
  }

  constexpr Node<floating_point_t> const *get_best_node() const noexcept {
    std::size_t best_node_index{};
    floating_point_t best_node_distance{};
    for (std::size_t iter = 0; iter < current_tree_size; ++iter) {
      if (best_node_distance < node_array[iter].location.distance()) {
        best_node_distance = node_array[iter].location.distance();
        best_node_index = iter;
      }
    }
    return node_array + best_node_index;
  }

  constexpr void insert_node(opt::Point<floating_point_t> const &point,
                             Node<floating_point_t> const *parent) noexcept {
    node_array[current_tree_size++] = Node<floating_point_t>{point, parent};
  }

 private:
  std::size_t current_tree_size;
  Node<floating_point_t> node_array[max_iterations + 1];  // zero node
};

}  // namespace opt::internal
