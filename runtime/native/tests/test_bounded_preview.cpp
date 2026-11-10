#define main sprite_transport_main
#include "../src/sprite_can_shadow.cpp"
#undef main

void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

struct Fixture {
  std::vector<Motor> motors = std::vector<Motor>(kJointCount);
  NativeKinematics kinematics;
  IpcSocket ipc;
  Fixture() {
    ipc.has_target = true;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      auto& m = motors[i];
      m.name = "test_motor_" + std::to_string(i);
      m.position_min = -12.5; m.position_max = 12.5;
      m.velocity_min = -20; m.velocity_max = 20;
      m.torque_min = -28; m.torque_max = 28;
      m.soft_position_min = -1; m.soft_position_max = 1;
      m.hard_position_min = -1.2; m.hard_position_max = 1.2;
      m.deployment_velocity_max = 10;
      m.commissioning_torque_cap = 28;
      m.mechanical_peak_torque = 40;
      kinematics.joint_to_motor[i][i] = 1;
      kinematics.motor_to_joint[i][i] = 1;
      kinematics.joint_effort_limit[i] = 40;
    }
  }
  void preview(bool bounded) {
    preview_final_motor_commands(motors, ipc, kinematics, bounded);
  }
};

int main() {
  Fixture quantized;
  auto& qm = quantized.motors[0];
  qm.velocity_min = -50; qm.velocity_max = 50;
  qm.torque_min = -10; qm.torque_max = 10;
  qm.commissioning_torque_cap = 1;
  qm.last_position = decode_uint(encode_uint(-0.5, -12.5, 12.5, 16), -12.5, 12.5, 16);
  qm.last_velocity = decode_uint(encode_uint(0, -50, 50, 12), -50, 50, 12);
  qm.preview_initialized = true;
  qm.preview_kp = 7; qm.preview_kd = 3;
  qm.preview_position = qm.last_position + (-0.98 + 3 * qm.last_velocity) / 7;
  bool quantization_rejected = false;
  try { (void)protected_policy_payload(qm); }
  catch (const std::runtime_error&) { quantization_rejected = true; }
  require(quantization_rejected, "must reproduce historical 7/3 quantization failure");
  (void)protected_policy_payload(qm, true);
  require(qm.maximum_abs_commanded_torque <= 1, "encoded correction exceeded cap");
  // Cover both signs, changed feedback after preview, and several gain/cap ranges.
  for (double cap : {0.08, 0.35, 1.0, 3.0, 10.0}) {
    for (double kd : {0.45, 0.5, 3.0, 5.0}) {
      for (int sign : {-1, 1}) {
        qm.commissioning_torque_cap = cap;
        qm.preview_kd = kd;
        qm.preview_position = -0.6;
        qm.last_velocity = sign * 0.75;
        qm.maximum_abs_commanded_torque = 0;
        (void)protected_policy_payload(qm, true);
        require(qm.maximum_abs_commanded_torque <= cap, "sweep encoded cap");
      }
    }
  }
  qm.commissioning_torque_cap = 0.00001;
  qm.preview_kp = 0; qm.preview_kd = 0;
  bool infeasible_rejected = false;
  try { (void)protected_policy_payload(qm, true); }
  catch (const std::runtime_error&) { infeasible_rejected = true; }
  require(infeasible_rejected, "unrepresentable zero torque must fail closed");

  // Reproduce the stationary knee's full-gain preview exceeding protocol TMAX.
  Fixture knee;
  knee.motors[0].last_position = -0.144;
  knee.ipc.latest_target.position_rad[0] = -0.367;
  knee.ipc.latest_target.kp[0] = 130;
  bool rejected = false;
  try { knee.preview(false); } catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "unbounded knee should reproduce failure");
  knee.ipc.motor_preview_count = 0;
  knee.preview(true);
  (void)protected_policy_payload(knee.motors[0]);
  require(knee.motors[0].maximum_abs_commanded_torque <= 28, "knee cap");

  Fixture fixed_gain;
  use_peak_pd_command_caps(fixed_gain.motors);
  auto& pd = fixed_gain.motors[0];
  require(pd.commissioning_torque_cap == 36. && pd.feedback_torque_cap == 0.,
          "90 percent changes only command cap");
  pd.preview_initialized = true;
  pd.preview_position = .75; pd.preview_kp = 130.; pd.preview_kd = 3.;
  pd.last_position = .1; pd.last_velocity = .4;
  const auto packet = protected_policy_payload(pd, false, true);
  require(static_cast<std::uint32_t>((packet[3] & 0xf) << 8 | packet[4]) ==
          encode_uint(130., 0., 500., 12), "preserve encoded Kp");
  require(static_cast<std::uint32_t>(packet[5] << 4 | packet[6] >> 4) ==
          encode_uint(3., 0., 5., 12), "preserve encoded Kd");
  require(static_cast<std::uint32_t>((packet[6] & 0xf) << 8 | packet[7]) ==
          encode_uint(0., -28., 28., 12), "preserve zero effort field");
  require(pd.maximum_abs_commanded_torque > 35.9 && pd.maximum_abs_commanded_torque <= 36.,
          "position-mode total PD may exceed the explicit torque field range");
  pd.preview_position = -.75;
  pd.maximum_abs_commanded_torque = 0.;
  (void)protected_policy_payload(pd, false, true);
  require(pd.maximum_abs_commanded_torque > 35.9 && pd.maximum_abs_commanded_torque <= 36.,
          "position saturation must also cover negative torque");
  pd.soft_position_max = -.2;
  rejected = false;
  try { (void)protected_policy_payload(pd, false, true); } catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "infeasible position-only saturation must reject the command");

  auto a = fixed_gain.motors[1], b = fixed_gain.motors[2];
  a.name = "right_ankle_motor_a";
  a.torque_min = -10.; a.torque_max = 10.;
  a.preview_feedforward_torque = 16.; b.preview_feedforward_torque = -8.;
  a.commissioning_torque_cap = 10.;
  limit_pd_ankle_pair(a,b);
  require(a.preview_feedforward_torque < 10. && a.preview_feedforward_torque > 9.9 &&
          b.preview_feedforward_torque == -.5*a.preview_feedforward_torque,
          "paired ankle saturation must preserve the torque ratio");

  // Strong damping at a joint limit must not move the target outside that limit.
  Fixture elbow;
  auto& m = elbow.motors[0];
  m.soft_position_min = -0.125;
  m.last_position = -0.05; m.last_velocity = -10;
  m.commissioning_torque_cap = 0.35;
  elbow.ipc.latest_target.position_rad[0] = -0.125;
  elbow.ipc.latest_target.kp[0] = 0.8882644;
  elbow.ipc.latest_target.kd[0] = 0.0565487;
  elbow.preview(true);
  require(m.preview_position == -0.125, "elbow target moved outside envelope");
  require(m.preview_kp < 0.8882644, "elbow impedance not reduced");
  (void)protected_policy_payload(m);
  require(m.maximum_abs_commanded_torque <= 0.35, "elbow quantized cap");

  Fixture invalid;
  invalid.ipc.latest_target.position_rad[0] = 1.3;
  rejected = false;
  try { invalid.preview(true); } catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "bounded preview must retain position guard");

  Fixture ankle;
  ankle.motors[0].name = "left_ankle_motor_a";
  ankle.kinematics.joint_names[0] = "left_ankle_pitch_joint";
  ankle.motors[0].commissioning_torque_cap = 10;
  ankle.ipc.latest_target.position_rad[0] = 0.5;
  ankle.ipc.latest_target.kp[0] = 80;
  ankle.preview(true);
  require(std::abs(ankle.motors[0].preview_feedforward_torque - 9.8) < 1e-9,
          "ankle feedforward cap");
  (void)protected_policy_payload(ankle.motors[0]);
  const auto now = monotonic_ns();
  require_supermimic_deadline(now, now + kSuperMimicDeadlineNs);
  rejected = false;
  try { require_supermimic_deadline(now, now + kSuperMimicDeadlineNs + 1); }
  catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "strict state deadline must reject one ns beyond 20 ms");
  int peer[2]{};
  require(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, peer) == 0, "state test socketpair");
  Fixture stamped;
  stamped.ipc.client_fd = peer[0];
  for (auto& motor : stamped.motors) motor.feedback_ns = now - 1'000'000;
  stamped.motors.back().feedback_ns = now - 4'000'000;
  send_state_packet(stamped.ipc, stamped.motors, 123, now, true);
  StatePacket state{};
  require(recv(peer[1], &state, sizeof(state), 0) == sizeof(state), "receive state test packet");
  require(state.monotonic_ns == now - 4'000'000 && stamped.ipc.source_feedback_ns == state.monotonic_ns,
          "published state must retain oldest kernel feedback timestamp");
  require(stamped.ipc.timing.size() == 1 && stamped.ipc.timing[0].publish_ns == now &&
          stamped.ipc.timing[0].oldest_motor == 30, "trace must identify oldest motor");
  TargetPacket target{};
  std::memcpy(target.magic, "SPRT", 4);
  target.version = kIpcVersion;
  target.kind = kTargetKind;
  target.sequence = 1;
  target.source_state_sequence = state.sequence;
  target.joint_order_hash = 456;
  target.monotonic_ns = monotonic_ns();
  target.feedforward_torque_nm[10] = .349;
  require(send(peer[1], &target, sizeof(target), 0) == sizeof(target), "send nonzero joint feedforward");
  rejected = false;
  try { drain_target_packets(stamped.ipc, 456, {}, monotonic_ns(), true); }
  catch (const std::runtime_error& e) {
    rejected = std::string(e.what()) == "SuperMimic joint feedforward must be zero";
  }
  require(rejected && stamped.ipc.target_count == 0, "strict IPC must reject added torque");
  target.feedforward_torque_nm[10] = 0.;
  require(send(peer[1], &target, sizeof(target), 0) == sizeof(target), "send pure PD target");
  drain_target_packets(stamped.ipc, 456, {}, monotonic_ns(), true);
  require(stamped.ipc.target_count == 1, "strict IPC must accept zero joint feedforward");
  close(peer[0]); close(peer[1]);
  std::vector<Motor> scheduled(16);
  for (std::size_t i=0; i<scheduled.size(); ++i) {
    auto& motor=scheduled[i];
    motor.interface=i<8 ? "legs" : "upper";
    motor.poll_rate_hz=i<2 ? 500 : 50;
    motor.poll_phase=i<2 ? static_cast<int>(2*i) : 1+2*static_cast<int>(i<8 ? i-2 : i-8);
    motor.cycle_poll_ns=now;
    motor.feedback_ns=now+500'000;
  }
  pack_supermimic_polls(scheduled);
  require(scheduled[0].poll_phase==0 && scheduled[1].poll_phase==2 &&
          scheduled[7].poll_phase==11 && scheduled[15].poll_phase==8,
          "packing must preserve ankle bus spacing and finish the upper bus earlier");
  scheduled.back().feedback_ns=now-1;
  require(first_missing_policy_feedback(scheduled,now)==&scheduled.back(),
          "a previous batch sample must block publication");
  scheduled.back().feedback_ns=now+1;
  require(first_missing_policy_feedback(scheduled,now)==nullptr,
          "a fresh late reply can complete the batch");
  auto accepts_duration = [](const char* duration, const char* acknowledgement,
                             const char* priority = "50", bool sling = false,
                             const char* walk_option = nullptr) {
    std::vector<std::string> args{
        "test", "--supermimic-commissioning", "--policy-actuation", "--config", "unused",
        "--output", "unused", "--ipc-socket", "unused", "--policy-joint-hash", "1",
        "--kinematics-config", "unused", "--joint-safety-config", "unused",
        "--all-motors-disabled-confirmed", "--duration", duration,
        "--realtime-priority", priority,
        "--acknowledge-hardware-tx", acknowledgement};
    if (sling) args.insert(args.end(), {"--tracked-startup", "--sling-ready-fd", "3", "--operator-status-fd", "4"});
    if (walk_option) args.push_back(walk_option);
    std::vector<char*> argv;
    for (auto& arg : args) argv.push_back(arg.data());
    try { (void)parse_options(static_cast<int>(argv.size()), argv.data()); }
    catch (const std::runtime_error&) { return false; }
    return true;
  };
  constexpr auto short_ack = "ENABLE_SUSPENDED_SUPERMIMIC_LOW_GAIN";
  constexpr auto six_ack = "ENABLE_SUSPENDED_SUPERMIMIC_LOW_GAIN_6S";
  constexpr auto ten_ack = "ENABLE_SUSPENDED_SUPERMIMIC_LOW_GAIN_10S";
  require(accepts_duration("2", short_ack) && accepts_duration("6", six_ack) &&
          accepts_duration("10", ten_ack), "valid 2, 6 and 10 second commissioning tiers");
  require(!accepts_duration("6", short_ack) && !accepts_duration("2", six_ack) &&
          !accepts_duration("7", six_ack), "six-second token must not admit a different tier");
  require(!accepts_duration("10", six_ack) && !accepts_duration("6", ten_ack) &&
          !accepts_duration("11", ten_ack), "ten-second token must not admit a different tier");
  require(!accepts_duration("10", ten_ack, "0"), "powered SuperMimic requires verified real-time scheduling");
  constexpr auto sling_ack = "ENABLE_SUPERMIMIC_ZERO_SPD_SLING_WINDOW";
  require(accepts_duration("30.5", sling_ack, "50", true) &&
          !accepts_duration("30.5", sling_ack) &&
          !accepts_duration("30.5", ten_ack, "50", true) &&
          !accepts_duration("22.5", sling_ack, "50", true) &&
          !accepts_duration("30.5", sling_ack, "0", true),
          "sling window requires its exact duration, token, channels and real-time scheduling");
  constexpr auto walk_ack = "ENABLE_SUPERMIMIC_WALK_020_10S";
  require(accepts_duration("49.5", walk_ack, "50", true, "--walk-020") &&
          !accepts_duration("49.5", walk_ack, "50", false, "--walk-020") &&
          !accepts_duration("49.5", sling_ack, "50", true, "--walk-020") &&
          !accepts_duration("30.5", walk_ack, "50", true, "--walk-020"),
          "walk-020 is isolated to its authorized profile and channels");
  constexpr auto walk030_ack = "ENABLE_SUPERMIMIC_WALK_030_10S";
  require(accepts_duration("49.5", walk030_ack, "50", true, "--walk-030") &&
          !accepts_duration("49.5", walk_ack, "50", true, "--walk-030") &&
          !accepts_duration("49.5", walk030_ack, "50", true, "--walk-020"),
          "walk-030 requires its own acknowledgement");
  constexpr auto walk025_ack = "ENABLE_SUPERMIMIC_WALK_025_10S";
  require(accepts_duration("49.5", walk025_ack, "50", true, "--walk-025") &&
          !accepts_duration("49.5", walk030_ack, "50", true, "--walk-025") &&
          !accepts_duration("49.5", walk025_ack, "50", true, "--walk-030") &&
          !accepts_duration("49.5", walk025_ack, "50", false, "--walk-025"),
          "walk-025 requires its own acknowledgement and unchanged readiness channels");
  using sprite_policy::sling_notice_elapsed;
  require(!sling_notice_elapsed(0, 3'000'000'000LL) &&
          !sling_notice_elapsed(1'000'000'000LL, 2'999'999'999LL) &&
          sling_notice_elapsed(1'000'000'000LL, 3'000'000'000LL),
          "policy ramp requires at least two seconds from the enable notice");
  using sprite_policy::sling_ready;
  require(sling_ready({1000, 50, 50}, 1000) &&
          !sling_ready({999, 50, 50}, 1000) &&
          !sling_ready({1000, 50, 50}, 999) &&
          !sling_ready({1000, 49, 50}, 1000) &&
          !sling_ready({1000, 50, 49}, 1000),
          "sling window requires a current accepted target and a full second of joint and body settling");
  require(sling_ready({1500, 50, 50}, 1500, 1500) &&
          !sling_ready({1000, 50, 50}, 1500, 1500) &&
          !sling_ready({1500, 49, 50}, 1500, 1500),
          "forward motion requires a separate current standing checkpoint");
  require(sling_ready({1000, 0, 50}, 1000, 1000, false) &&
          sling_ready({1500, 0, 50}, 1500, 1500, false) &&
          !sling_ready({1500, 50, 49}, 1500, 1500, false) &&
          !sling_ready({1500, 0, 50}, 1499, 1500, false),
          "walk readiness ignores joint settling and still requires current body stability");
  using sprite_policy::walk_vx;
  for (const double speed : {.20, .25, .30}) {
    std::size_t plateau = 0;
    for (std::uint64_t sequence = 1; sequence <= 2476; ++sequence) {
      const double vx = walk_vx(sequence, speed);
      require(vx >= 0. && vx <= speed, "forward command is bounded");
      plateau += vx == speed;
    }
    require(walk_vx(1502, speed) == 0 && walk_vx(1503, speed) > 0 &&
            walk_vx(1651, speed) < speed && walk_vx(1652, speed) == speed &&
            walk_vx(2151, speed) == speed && walk_vx(2152, speed) < speed &&
            walk_vx(2301, speed) == 0 && plateau == 500,
            "walking keeps the zero start, two ramps and exactly ten seconds at commanded speed");
  }
  std::cout << "PASS bounded knee, elbow damping, mechanical envelope, ankle, MIT quantization\n";
}
