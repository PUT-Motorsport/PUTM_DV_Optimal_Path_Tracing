#pragma once

#include <random>
#include <type_traits>

namespace opt::random {

template <typename floating_point_t>
class Gaussian {
 public:
  constexpr Gaussian() = default;
  constexpr Gaussian(floating_point_t mean, floating_point_t std_dev) noexcept
      : random_engine((std::random_device())()), normal_dist(mean, std_dev) {}
  constexpr floating_point_t get() noexcept {
    return normal_dist(random_engine);
  }

 private:
  std::mt19937 random_engine;
  std::normal_distribution<floating_point_t> normal_dist;
};

}  // namespace opt::random
