#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace sprite_policy {
using ProtectionVector = std::array<double, 31>;
struct ImpedanceTarget {
    ProtectionVector position{}, velocity{}, kp{}, kd{}, feedforward{};
};
inline void require_finite(const ProtectionVector& v) {
    for (double x : v) if (!std::isfinite(x)) throw std::runtime_error("nonfinite target");
}
inline void validate_target(const ImpedanceTarget& t) {
    require_finite(t.position); require_finite(t.velocity);
    require_finite(t.kp); require_finite(t.kd); require_finite(t.feedforward);
    for (std::size_t i=0; i<31; ++i)
        if (t.kp[i]<0 || t.kd[i]<0) throw std::runtime_error("negative gain");
}
class TargetProjector {
    ProtectionVector lower_, upper_, gain_, damping_;
    double scale_, maximum_kd_;
public:
    std::array<std::uint64_t,31> clamp_count{};
    double maximum_overshoot=0;
    TargetProjector(ProtectionVector lower, ProtectionVector upper,
                    ProtectionVector gain, ProtectionVector damping,
                    double scale, double maximum_kd)
        : lower_(lower),upper_(upper),gain_(gain),damping_(damping),
          scale_(scale),maximum_kd_(maximum_kd) {
        require_finite(lower); require_finite(upper); require_finite(gain); require_finite(damping);
        if (!std::isfinite(scale) || scale<=0 || scale>1 ||
            !std::isfinite(maximum_kd) || maximum_kd<=0) throw std::runtime_error("invalid gain envelope");
        for (std::size_t i=0;i<31;++i)
            if (lower[i]>=upper[i] || gain[i]<=0 || gain[i]>1 || damping[i]<=0 || damping[i]>4)
                throw std::runtime_error("invalid projection configuration");
    }
    ProtectionVector require_position(const ProtectionVector& p) const {
        require_finite(p);
        for (std::size_t i=0;i<31;++i)
            if (p[i]<lower_[i] || p[i]>upper_[i]) throw std::runtime_error("startup pose outside limits");
        return p;
    }
    ImpedanceTarget project(const ImpedanceTarget& t) {
        validate_target(t);
        auto result=t;
        for (std::size_t i=0;i<31;++i) {
            result.position[i]=std::clamp(t.position[i],lower_[i],upper_[i]);
            if (result.position[i]!=t.position[i]) ++clamp_count[i];
            maximum_overshoot=std::max(maximum_overshoot,
                std::max(lower_[i]-t.position[i],t.position[i]-upper_[i]));
            const double s=scale_*gain_[i];
            result.kp[i]=s*t.kp[i];
            result.kd[i]=(s*damping_[i])*t.kd[i];
            result.feedforward[i]=s*t.feedforward[i];
            if (result.kd[i]>maximum_kd_) throw std::runtime_error("projected Kd exceeds qualified limit");
        }
        validate_target(result);
        return result;
    }
};
class StartupRamp {
    std::uint64_t hold_, ramp_, completed_=0;
    ProtectionVector initial_{}, previous_{};
    bool captured_=false, has_previous_=false;
public:
    double last_alpha=0, maximum_position_delta=0;
    StartupRamp(double hz,double hold_seconds,double ramp_seconds) {
        if (!std::isfinite(hz) || hz<=0 || !std::isfinite(hold_seconds) || hold_seconds<0 ||
            !std::isfinite(ramp_seconds) || ramp_seconds<=0)
            throw std::runtime_error("invalid startup timing");
        const double h=std::ceil(hz*hold_seconds), r=std::ceil(hz*ramp_seconds);
        // Bound counters before converting floating-point configuration to integers.
        if (!std::isfinite(h) || !std::isfinite(r) || h>1e9 || r>1e9)
            throw std::runtime_error("startup duration too large");
        hold_=static_cast<std::uint64_t>(h);
        ramp_=std::max<std::uint64_t>(1,static_cast<std::uint64_t>(r));
    }
    bool captured() const { return captured_; }
    bool complete() const { return completed_>=hold_+ramp_; }
    ImpedanceTarget apply(const ImpedanceTarget& t,const ProtectionVector& measured) {
        require_finite(measured); validate_target(t);
        if (!captured_) { initial_=measured; captured_=true; }
        double a=0;
        if (completed_>=hold_) {
            const double linear=std::min(1.0,static_cast<double>(completed_-hold_+1)/ramp_);
            a=linear*linear*(3-2*linear);
        }
        auto result=t;
        for (std::size_t i=0;i<31;++i) {
            result.position[i]=(1-a)*initial_[i]+a*t.position[i];
            result.velocity[i]=a*t.velocity[i]; result.kp[i]=a*t.kp[i];
            result.kd[i]=a*t.kd[i]; result.feedforward[i]=a*t.feedforward[i];
            if (has_previous_) maximum_position_delta=std::max(maximum_position_delta,
                std::abs(result.position[i]-previous_[i]));
        }
        validate_target(result);
        previous_=result.position; has_previous_=true; last_alpha=a; ++completed_;
        return result;
    }
};

enum class StartupJoint : std::uint8_t {
    arm, stance, left_ankle, right_ankle, hold
};

// Smoothly hand over the measured pose to the projected policy target. The
// settling check is diagnostic and does not gate the command or gain ramp.
class TrackedStartup {
    static constexpr double settle_tracking_error_rad_=0.10;
    static constexpr double settle_speed_rad_s_=0.30;
    static constexpr unsigned settle_frames_=50;
    StartupRamp ramp_;
    std::array<StartupJoint,31> roles_;
    bool follow_all_ = false;
  bool direct_=false, direct_started_=false;
    ProtectionVector initial_{};
    unsigned stable_frames_=0;
    bool ankle_joint(std::size_t j) const {
        return roles_[j]==StartupJoint::left_ankle || roles_[j]==StartupJoint::right_ankle;
    }
    bool stance_joint(std::size_t j) const {
        return follow_all_ || roles_[j]==StartupJoint::stance || ankle_joint(j);
    }
public:
    double tracking_error_rad=0., goal_error_rad=0.;
    explicit TrackedStartup(std::array<StartupJoint,31> roles, double hold_seconds=1., bool follow_all=false,
                            bool direct=false)
        : ramp_(50,hold_seconds,4.), roles_(roles), follow_all_(follow_all), direct_(direct) {
        if (direct_ && !follow_all_) throw std::runtime_error("direct start requires full-body control");
        std::array<unsigned,5> counts{};
        for (const auto joint : roles_) {
            const auto role=static_cast<std::size_t>(joint);
            if (role>=counts.size()) throw std::runtime_error("invalid startup joint role");
            ++counts[role];
        }
        if (counts!=std::array<unsigned,5>{8,10,2,2,9})
            throw std::runtime_error("unexpected SuperMimic startup joint groups");
    }
    bool captured() const { return direct_?direct_started_:ramp_.captured(); }
    bool complete() const { return direct_?direct_started_:ramp_.complete(); }
    double alpha() const { return direct_?(direct_started_?1.:0.):ramp_.last_alpha; }
    unsigned stable_frames() const { return stable_frames_; }
    bool settled() const { return stable_frames_==settle_frames_; }

    ImpedanceTarget apply(const ImpedanceTarget& goal, const ProtectionVector& measured,
                          const ProtectionVector& velocity) {
        require_finite(velocity);
        if (!captured()) initial_=measured;
        auto target=goal;
        target.velocity={};
        target.feedforward={};
        for (std::size_t j=0; j<measured.size(); ++j)
            if (!follow_all_ && roles_[j]==StartupJoint::hold) target.position[j]=initial_[j];
        auto result=direct_?target:ramp_.apply(target,measured);
        if (direct_) direct_started_=true;
        // Preserve the captured angles exactly for the independently held joints.
        for (std::size_t j=0; j<measured.size(); ++j)
            if (!follow_all_ && roles_[j]==StartupJoint::hold) result.position[j]=initial_[j];
        tracking_error_rad=0.;
        goal_error_rad=0.;
        bool quiet=true;
        for (std::size_t j=0; j<measured.size(); ++j) {
            quiet=quiet && std::abs(velocity[j])<=settle_speed_rad_s_;
            if (stance_joint(j)) {
                goal_error_rad=std::max(goal_error_rad,std::abs(goal.position[j]-measured[j]));
                tracking_error_rad=std::max(tracking_error_rad,std::abs(result.position[j]-measured[j]));
            }
        }
        stable_frames_=complete() && quiet && goal_error_rad<=settle_tracking_error_rad_ ?
            std::min(stable_frames_+1,settle_frames_) : 0;
        return result;
    }
};
}
