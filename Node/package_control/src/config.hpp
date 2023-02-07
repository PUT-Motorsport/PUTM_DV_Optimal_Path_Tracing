#pragma once

namespace Control::Config {
constexpr auto target_velocity{1.0}; // in m/s
// Lateral
constexpr auto lookahead_distance{4.0};
constexpr auto max_steering_angle{25.0}; // todo: find value in simulator
constexpr auto longitudinal_control_loop_frequency{10};
} // namespace Control::Config
