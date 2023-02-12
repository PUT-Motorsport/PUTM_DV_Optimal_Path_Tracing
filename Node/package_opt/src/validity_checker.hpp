#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <cmath>
#include <numeric>

#include "config.hpp"
#include "opt_assert.hpp"

/*
The OMPL must be provided a validity checker.
The validity checker is informed about the positons of cones, and checks whether
a state intersects with any of them
*/
namespace opt {

template <typename T>
class Obstacle2D {
 public:
  explicit constexpr Obstacle2D() = default;
  virtual ~Obstacle2D() = default;
  virtual T clearance(T x, T) const = 0;
};

template <typename T>
class ValidityChecker : public ompl::base::StateValidityChecker {
 public:
  explicit ValidityChecker(
      const ompl::base::SpaceInformationPtr &space_information)
      : ompl::base::StateValidityChecker(space_information), obstacles() {
    static_assert(std::is_floating_point_v<T>);
  }

  explicit ValidityChecker(
      const ompl::base::SpaceInformationPtr &space_information,
      package_opt::Cones::ConstPtr const &cones)
      : ompl::base::StateValidityChecker(space_information), obstacles(cones) {
    static_assert(std::is_floating_point_v<T>);
    opt_assert(cones->cones_x.size() == cones->cones_y.size());
  }

  // return the lowest clearance to all objects (cones)
  [[nodiscard]] bool isValid(const ompl::base::State *state) const override {
    return this->clearance(state) > 0.0;
  }
  [[nodiscard]] double clearance(
      const ompl::base::State *state) const override {
    const auto x_pos =
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    const auto y_pos =
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    T lowest_clearance{std::numeric_limits<T>::max()};

    for (std::size_t iter = 0; iter < obstacles->cones_x.size(); ++iter) {
      const auto clearance = std::hypot(obstacles->cones_x[iter] - x_pos,
                                        obstacles->cones_y[iter] - y_pos);
      if (clearance < lowest_clearance) {
        lowest_clearance = clearance;
      }
    }

    return lowest_clearance - config::cone_radius;
  }

 private:
  package_opt::Cones::ConstPtr obstacles;
};

ompl::base::OptimizationObjectivePtr getPathLengthObjective(
    const ompl::base::SpaceInformationPtr &space_information) {
  ompl::base::OptimizationObjectivePtr objective(
      new ompl::base::PathLengthOptimizationObjective(space_information));
  objective->setCostThreshold(
      ompl::base::Cost(config::track_optimality_threshold));
  return objective;
}

}  // namespace opt
