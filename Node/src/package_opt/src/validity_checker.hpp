#pragma once

#include <ompl-1.5/ompl/base/StateValidityChecker.h>
#include <ompl-1.5/ompl/base/SpaceInformation.h>
#include <ompl-1.5/ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl-1.5/ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "opt_assert.hpp"
#include "config.hpp"

#include <cmath>
#include <numeric>

/*
The OMPL must be provided a validity checker, which checks whether a state is valid.
The validity checker is informed about the positons of cones, and checks whether a state intersects with any of them
*/
namespace opt {

    template<typename T>
    class Obstacle2D {
    public:
        explicit constexpr Obstacle2D() = default;
        virtual ~Obstacle2D() = default;
        virtual T clearance(T x, T) const = 0;
    };

    template<typename T>
    class ValidityChecker : public ompl::base::StateValidityChecker {
    public:
        explicit ValidityChecker(const ompl::base::SpaceInformationPtr& space_information)
            : ompl::base::StateValidityChecker(space_information), obstacles() {
            static_assert(std::is_floating_point_v<T>);
        }

        explicit ValidityChecker(const ompl::base::SpaceInformationPtr& space_information, package_opt::Cones::ConstPtr const& cones) :
                ompl::base::StateValidityChecker(space_information), obstacles(cones) {
            static_assert(std::is_floating_point_v<T>);
            opt_assert(cones->cones_left_x.size() == cones->cones_left_y.size());
            opt_assert(cones->cones_right_x.size() == cones->cones_right_y.size());
        }

        //return the lowest clearance to all objects (cones)
        [[nodiscard]] bool isValid(const ompl::base::State * state) const override {
            return this->clearance(state) > 0.0;
        }
        [[nodiscard]] double clearance(const ompl::base::State * state) const override {
            const auto x_pos = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            const auto y_pos = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
            T lowest_clearance{std::numeric_limits<T>::max()};

            for (std::size_t iter = 0; iter < obstacles->cones_left_x.size(); ++iter) {
                const auto clearance = std::hypot(obstacles->cones_left_x[iter] - x_pos,
                                                  obstacles->cones_left_y[iter] - y_pos);
                if (clearance < lowest_clearance) {
                    lowest_clearance = clearance;
                }
            }
            for (std::size_t iter = 0; iter < obstacles->cones_right_x.size(); ++iter) {
                const auto clearance = std::hypot(obstacles->cones_right_x[iter] - x_pos,
                                                  obstacles->cones_right_y[iter] - y_pos);
                if (clearance < lowest_clearance) {
                    lowest_clearance = clearance;
                }
            }
            return lowest_clearance - config::cone_radius;
        }

    private:
        package_opt::Cones::ConstPtr obstacles;

    };

    ompl::base::OptimizationObjectivePtr getPathLengthObjective(const ompl::base::SpaceInformationPtr& space_information) {
        return ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(space_information));
    }

    template<typename T>
    class [[deprecated("Left for reference as an alternative solution")]] Cone : public Obstacle2D<T> {
    public:
        explicit constexpr Cone(T x, T y) : cone_x(x), cone_y(y), cone_radius(config::cone_radius) {
            static_assert(std::is_floating_point_v<T>);
        }

        T clearance(T xPos, T yPos) const override {
            const auto delta_x = xPos - cone_x;
            const auto delta_y = yPos - cone_y;

            return std::hypot(delta_x, delta_y) - cone_radius;
        }
    private:
        T cone_x;
        T cone_y;
        T cone_radius;
    };
}   //namespace opt
