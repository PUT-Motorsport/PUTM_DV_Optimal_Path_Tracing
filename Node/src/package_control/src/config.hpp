#pragma once

namespace Control::Config {
// Longitudinal
constexpr auto control_loop_frequency = 10; // todo: all values tbd
constexpr auto Kp = 0.5;
constexpr auto Ki = 0.1;
constexpr auto Kd = 0.;
constexpr auto target_velocity{2.0}; // in m/s
// Lateral
constexpr auto lookahead_distance{5.0};
constexpr auto max_steering_angle{45.0}; // todo: find value in simulator
constexpr auto longitudinal_control_loop_frequency{10};
} // namespace Control::Config
