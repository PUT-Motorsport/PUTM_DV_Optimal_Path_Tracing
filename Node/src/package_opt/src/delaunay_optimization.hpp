#pragma once

#include <optional>
#include <ros/ros.h> //debug messages
#include <type_traits>
#include <vector>

#include "Delaunay/delaunay.h"
#include "common_defs.hpp"
#include "math_utility.hpp"
#include "package_opt/Cones.h"

namespace opt {
template <typename T>
std::vector<Point<T>>
delaunay_optimize_track(package_opt::Cones::ConstPtr const &cones,
                        std::vector<Point<T>> const &rrt_path) {
  static_assert(std::is_floating_point_v<T>);

  // Compute the delaunay triangulation
  std::vector<dt::Vector2<double>> points;
  points.reserve(cones->cones_x.size());
  for (std::size_t i = 0; i < cones->cones_x.size(); ++i) {
    points.emplace_back(cones->cones_x[i], cones->cones_y[i]);
  }
  dt::Delaunay<double> triangulation;
  triangulation.triangulate(points);
  const auto &triangles = triangulation.getTriangles();

  std::vector<opt::Point<T>> delaunay_points;
  delaunay_points.reserve(
      triangles.size() *
      2); // best case, every triangle will have two edges intersecting the path
  for (const auto &triangle : triangles) {
    for (std::size_t iter = 1; iter < rrt_path.size(); ++iter) {

      std::optional<Point<T>> intersection_point_a =
          utility::find_intersection_point(
              dt::Edge<T>(*triangle.a, *triangle.b), opt::Edge<T>{rrt_path[iter - 1], rrt_path[iter]});

      std::optional<Point<T>> intersection_point_b =
          utility::find_intersection_point(
              dt::Edge<T>(*triangle.b, *triangle.c), opt::Edge<T>{rrt_path[iter - 1], rrt_path[iter]});

      std::optional<Point<T>> intersection_point_c =
          utility::find_intersection_point(
              dt::Edge<T>(*triangle.a, *triangle.c), opt::Edge<T>{rrt_path[iter - 1], rrt_path[iter]});

      if (not(intersection_point_a.has_value() or
              intersection_point_b.has_value() or
              intersection_point_c.has_value())) {
        continue;
      }

      if (intersection_point_a.has_value()) {
        delaunay_points.emplace_back(
            Point<T>{(triangle.a->x + triangle.b->x) / 2,
                     (triangle.a->y + triangle.b->y) / 2});
      }

      if (intersection_point_b.has_value()) {
        delaunay_points.emplace_back(
            Point<T>{(triangle.b->x + triangle.c->x) / 2,
                     (triangle.b->y + triangle.c->y) / 2});
      }

      if (intersection_point_c.has_value()) {
        delaunay_points.emplace_back(
            Point<T>{(triangle.a->x + triangle.c->x) / 2,
                     (triangle.a->y + triangle.c->y) / 2});
      }
      break;
    }
  }
  ROS_INFO("Extrapolated %d points from %d triangles", (int)delaunay_points.size(),
           (int)triangles.size());
  return delaunay_points;
}
} // namespace opt
