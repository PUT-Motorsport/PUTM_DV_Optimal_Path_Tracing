#pragma once

#include <type_traits>

namespace Control {

template <typename T> class PID {
public:
  constexpr explicit PID(T kp, T ki, T kd, T dt);
  constexpr double update(T setpoint, T measurement) noexcept;
  void reset() noexcept;

private:
  T kp, ki, kd, dt;
  T integral, last_error;
};

template <typename T>
constexpr PID<T>::PID(T kp, T ki, T kd, T dt) : kp(kp), ki(ki), kd(kd), dt(dt) {
  static_assert(std::is_floating_point_v<T>);
  reset();
}

template <typename T>
constexpr double PID<T>::update(T setpoint, T measurement) noexcept {
  const T error = setpoint - measurement;
  integral += error * dt;
  const T derivative = (error - last_error) / dt;
  last_error = error;
  return kp * error + ki * integral + kd * derivative;
}

template <typename T> void PID<T>::reset() noexcept {
  integral = 0;
  last_error = 0;
}

} // namespace Control
