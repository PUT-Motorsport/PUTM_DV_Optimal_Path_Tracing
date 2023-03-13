#define CATCH_CONFIG_MAIN

#include "../Node/package_opt/src/Delaunay/edge.h"
#include "../Node/package_opt/src/Delaunay/vector2.h"
#include "../Node/package_opt/src/math_utility.hpp"
#include <catch2/catch_all.hpp>
#include <optional>

TEST_CASE("Line intersection tests") {
  opt::Point<double> a{1.0, 1.0};
  opt::Point<double> b{2.0, 2.0};
  opt::Point<double> c{1.0, 2.0};
  opt::Point<double> d{2.0, 1.0};
  std::optional intersection_point = opt::utility::find_intersection_point(
      dt::Edge<double>(dt::Vector2<double>(a.x, a.y),
                       dt::Vector2<double>(b.x, b.y)),
      opt::Edge<double>{c, d});

  REQUIRE(intersection_point.has_value());
  REQUIRE(intersection_point.value().x == 1.5);
  REQUIRE(intersection_point.value().y == 1.5);

  opt::Point<double> e{1.0, 1.0};
  opt::Point<double> f{3.0, 3.0};
  opt::Point<double> g{1.0, 2.0};
  opt::Point<double> h{3.0, 0.0};

  std::optional intersection_point2 = opt::utility::find_intersection_point(
      dt::Edge<double>(dt::Vector2<double>{e.x, e.y},
                       dt::Vector2<double>{f.x, f.y}),
      opt::Edge<double>{g, h});
  REQUIRE(intersection_point2.has_value());
}

TEST_CASE("Deals with parallel lines") {
  opt::Point<double> a{1.0, 1.0};
  opt::Point<double> b{2.0, 2.0};
  opt::Point<double> c{1.0, 2.0};
  opt::Point<double> d{2.0, 3.0};

  std::optional intersection_point3 = opt::utility::find_intersection_point(
      dt::Edge<double>(dt::Vector2<double>{a.x, a.y},
                       dt::Vector2<double>{b.x, b.y}),
      opt::Edge<double>{c, d});
  REQUIRE(not intersection_point3.has_value());
}

TEST_CASE("Deals with the same line") {
  opt::Point<double> a{1.0, 1.0};
  opt::Point<double> b{2.0, 2.0};

  std::optional intersection_point = opt::utility::find_intersection_point(
      dt::Edge<double>(dt::Vector2<double>{a.x, a.y},
                       dt::Vector2<double>{b.x, b.y}),
      opt::Edge<double>{a, b});
  REQUIRE(not intersection_point.has_value());
}

TEST_CASE("Random values") {
  opt::Edge<double> b{opt::Point<double>{0.0, 0.0},
                      opt::Point<double>{6.447265, 0.214151}};
  dt::Edge<double> a(dt::Vector2<double>{20.577024, -0.894882},
                     dt::Vector2<double>{15.091891, -0.598344});
  std::optional intersection_point =
      opt::utility::find_intersection_point(a, b);
  REQUIRE(not intersection_point.has_value());
}

TEST_CASE("In range tests") {
  REQUIRE_FALSE(opt::utility::is_in_range(1., 2., 3.));
  REQUIRE(opt::utility::is_in_range(1.0, 2.0, 1.5));
}
