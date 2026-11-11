#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace sprite_policy {
constexpr std::int64_t kFullRatePeriodNs = 2'000'000;
inline bool full_rate_due(std::size_t slot) { return slot % 4 == 0; }
inline std::size_t full_rate_trace_capacity(double seconds, std::size_t motors) {
  if (!std::isfinite(seconds) || seconds <= 0 || seconds > 120 || motors > 31)
    throw std::runtime_error("invalid full-rate evidence reservation");
  // Includes the maximum 2 s preparation, shutdown checks and fault margin.
  return static_cast<std::size_t>(std::ceil((seconds + 8.) * 500.)) * motors + 1024;
}
template<class Motors> std::vector<std::size_t> interleaved_motor_order(const Motors& motors, bool enabled=true) {
  std::vector<std::size_t> order(motors.size());
  std::iota(order.begin(), order.end(), 0);
  if (!enabled) return order;
  std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
    if (motors[a].can_id != motors[b].can_id) return motors[a].can_id < motors[b].can_id;
    return motors[a].interface < motors[b].interface;
  });
  return order;
}
}
