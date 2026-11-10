#pragma once

#include <cstdint>

namespace sprite_policy {
constexpr double kSlingStartupHoldSeconds = 2.4;
constexpr std::int64_t kSlingNoticeMinimumNs = 2'000'000'000LL;
constexpr int kSlingStandSeconds = 20;
constexpr int kSlingCountdownSeconds = 10;
// Leave half a second for delivery of the local operator notification.
constexpr double kSlingDurationSeconds = kSlingStandSeconds + kSlingCountdownSeconds + 0.5;
constexpr std::uint64_t kSlingReadySequence = kSlingStandSeconds * 50;
constexpr std::uint64_t kSlingStableFrames = 50;
constexpr int kWalkStartSeconds = kSlingStandSeconds + kSlingCountdownSeconds;
constexpr int kWalkFullSeconds = kWalkStartSeconds + 3;
constexpr int kWalkStopSeconds = kWalkFullSeconds + 10;
constexpr int kWalkZeroSeconds = kWalkStopSeconds + 3;
constexpr double kWalkDurationSeconds = kWalkZeroSeconds + 3.5;
constexpr std::uint64_t kWalkReadySequence = kWalkStartSeconds * 50;

inline double walk_vx(std::uint64_t sequence, double speed) {
    // The first state is the pre-enable state. The second begins the active timeline.
    const auto tick = sequence < 2 ? 0 : sequence - 2;
    constexpr auto start = kWalkStartSeconds * 50;
    constexpr auto full = kWalkFullSeconds * 50;
    constexpr auto stop = kWalkStopSeconds * 50;
    constexpr auto zero = kWalkZeroSeconds * 50;
    if (tick < start || tick >= zero) return 0.;
    if (tick >= full && tick < stop) return speed;
    if (tick < full) {
        const double x = static_cast<double>(tick - start) / (full - start);
        return speed * x * x * (3. - 2. * x);
    }
    // The first sample after the 500 full-speed samples starts deceleration.
    const double x = static_cast<double>(tick - stop + 1) / (zero - stop);
    return speed * (1. - x * x * (3. - 2. * x));
}

inline bool sling_notice_elapsed(std::int64_t notice_ns, std::int64_t now_ns) {
    return notice_ns > 0 && now_ns >= notice_ns && now_ns - notice_ns >= kSlingNoticeMinimumNs;
}

struct SlingReadiness {
    std::uint64_t sequence, joint_stable_frames, body_stable_frames;
};

inline bool sling_ready(const SlingReadiness& ready, std::uint64_t accepted_sequence,
                       std::uint64_t expected_sequence = kSlingReadySequence,
                       bool require_joint_settling = true) {
    return ready.sequence == expected_sequence && accepted_sequence >= ready.sequence &&
        (!require_joint_settling || ready.joint_stable_frames == kSlingStableFrames) &&
        ready.body_stable_frames == kSlingStableFrames;
}
}
