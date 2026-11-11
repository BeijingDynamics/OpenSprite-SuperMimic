#define main sprite_transport_main
#include "../src/sprite_can_shadow.cpp"
#undef main
#include <cassert>

int main() {
  using namespace sprite_policy;
  std::vector<Motor> motors(31);
  std::size_t index=0;
  for(int bus=1;bus<=4;++bus) for(int id=1;id<=(bus==4?7:8);++id) {
    auto& m=motors[index++];m.interface="kcan"+std::to_string(bus);m.can_id=id;
  }
  const auto order=interleaved_motor_order(motors);
  assert(order.size()==31);
  for(std::size_t rank=0;rank<28;++rank)
    assert(motors[order[rank]].interface=="kcan"+std::to_string(rank%4+1));
  std::size_t sends=0, commits=0;
  std::uint64_t active=1,pending=1;
  // Every action arrives between 500 Hz boundaries. It is committed once;
  // its identity is held without manufacturing new policy updates.
  for(std::size_t slot=0;slot<4000;++slot) {
    if(slot%40==13) ++pending;
    if(coherent_commit_due(slot,pending,active)) {active=pending;++commits;}
    if(full_rate_due(slot)) {++sends;assert(active==pending || slot%40<16);}
  }
  assert(sends==1000 && commits==100 && active==101);
  assert(full_rate_trace_capacity(49.5,8)>8*500*(49.5+2.2));
  Motor slow,fast;
  slow.poll_rate_hz=50;fast.poll_rate_hz=500;
  slow.temperature_filter_time_constant_s=fast.temperature_filter_time_constant_s=2.;
  update_temperature_filter(slow,20.,20.);update_temperature_filter(fast,20.,20.);
  for(int i=0;i<50;++i) update_temperature_filter(slow,40.,40.);
  for(int i=0;i<500;++i) update_temperature_filter(fast,40.,40.);
  assert(std::abs(slow.filtered_mos_temperature-fast.filtered_mos_temperature)<1e-10);
  for(int i=0;i<5000;++i) update_torque_rms_monitor(fast,2.);
  assert(fast.short_torque_squared_samples.size()==500);
  assert(fast.long_torque_squared_samples.size()==5000);
  assert(fast.maximum_short_torque_rms==2. && fast.maximum_long_torque_rms==2.);
  Motor delayed;
  delayed.cadence_monitors=true;
  delayed.full_rate_monitors=true;
  delayed.monitor_epoch_ns=1'000'000;
  delayed.monitor_poll_ns=3'000'000;
  delayed.nominal_sends=2;
  assert(!admit_monitor_sample(delayed,999'999));
  // Two replies are drained together after the second send. Both must feed
  // the unchanged 1 s/10 s RMS and temperature monitoring, not just the latter.
  assert(admit_monitor_sample(delayed,2'500'000));
  assert(admit_monitor_sample(delayed,3'500'000));
  assert(delayed.monitor_samples==2);
  assert(!admit_monitor_sample(delayed,3'600'000));
  ++delayed.nominal_sends;
  assert(admit_monitor_sample(delayed,5'500'000));
  Motor mixed;
  mixed.cadence_monitors=true;
  mixed.monitor_poll_ns=3'000'000;
  assert(!admit_monitor_sample(mixed,2'500'000));
  assert(admit_monitor_sample(mixed,3'500'000));
  assert(!admit_monitor_sample(mixed,3'600'000));
  assert(mixed.monitor_samples==1);
  // Completion EOF must not depend on serialization of the recorded evidence.
  int pair[2];
  assert(socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, pair)==0);
  IpcSocket ipc;
  ipc.client_fd=pair[0];
  close_ipc(ipc);
  pollfd finished{pair[1],POLLIN,0};
  assert(poll(&finished,1,0)==1);
  unsigned char byte=0;
  assert(recv(pair[1],&byte,1,MSG_DONTWAIT)==0);
  assert(ipc.client_fd==-1 && ipc.server_fd==-1);
  close_ipc(ipc);
  close(pair[1]);
  std::cout << "500 Hz schedule, sequence, physical monitor windows and completion EOF passed\n";
}
