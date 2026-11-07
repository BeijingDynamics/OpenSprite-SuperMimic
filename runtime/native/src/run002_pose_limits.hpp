#pragma once
// Frozen run002 conservative software limits. Not measured mechanical hard stops.
// Source hardware SHA256: 9ddf06034f24bbca49cb06f80236caa4cea7aee0040516d11209f848da1f2c10
// Generated in physical_motor_names order; a newer asset is a separate experiment.
#include <array>
#include <cstdint>
namespace sprite_run002 {
struct PoseLimit { const char* name; double lower,upper,speed; };
inline constexpr const char* hardware_sha256="9ddf06034f24bbca49cb06f80236caa4cea7aee0040516d11209f848da1f2c10";
inline constexpr const char* policy_sha256="f3529c927a44f970908d48e5694e63302d0fc838121aba2fa87c0a0863968bba";
inline constexpr std::uint64_t name_hash=1787374928349695527ULL;
inline constexpr std::array<PoseLimit,31> limits{{
    {"left_hip_pitch_motor",-2.74,2.74,9.3},
    {"right_hip_pitch_motor",-2.74,2.74,9.3},
    {"waist_roll_motor",-0.611,0.611,4.974188368183839},
    {"left_hip_roll_motor",-1.047,1.745,9.3},
    {"right_hip_roll_motor",-1.745,1.047,9.3},
    {"waist_yaw_motor",-2.618,2.618,9.3},
    {"left_hip_yaw_motor",-2.74,2.74,9.3},
    {"right_hip_yaw_motor",-2.74,2.74,9.3},
    {"left_shoulder_pitch_motor",-3.142,1.571,9.3},
    {"right_shoulder_pitch_motor",-1.571,3.142,9.3},
    {"left_knee_motor",-0.175,2.443,9.3},
    {"right_knee_motor",-2.443,0.175,9.3},
    {"left_shoulder_roll_motor",-0.175,2.792,9.3},
    {"right_shoulder_roll_motor",-2.792,0.175,9.3},
    {"head_yaw_motor",-1.745,1.745,50.0},
    {"left_shoulder_yaw_motor",-1.745,1.745,36.2},
    {"right_shoulder_yaw_motor",-1.745,1.745,36.2},
    {"left_elbow_motor",-2.618,0.175,36.2},
    {"right_elbow_motor",-0.175,2.618,36.2},
    {"left_wrist_yaw_motor",-1.745,1.745,36.2},
    {"right_wrist_yaw_motor",-1.745,1.745,36.2},
    {"left_wrist_pitch_motor",-0.785,0.785,50.0},
    {"right_wrist_pitch_motor",-0.785,0.785,50.0},
    {"left_wrist_roll_motor",-0.75,0.75,50.0},
    {"right_wrist_roll_motor",-0.75,0.75,50.0},
    {"left_ankle_motor_a",-1.0244126587327627,0.9953726587327622,36.2},
    {"left_ankle_motor_b",-0.9968932308136411,1.022893230813641,36.2},
    {"right_ankle_motor_a",-0.995811971186018,1.0247719711860181,36.2},
    {"right_ankle_motor_b",-1.0235823143033345,0.9971023143033343,36.2},
    {"head_motor_a",-1.0567680293388269,0.7944623604896802,50.0},
    {"head_motor_b",-0.7944623604896802,1.0567680293388264,50.0},
}};
}
