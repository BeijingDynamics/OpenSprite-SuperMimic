// Read-only replay of recorded joint states and policy targets through production PD encoding.
#define main sprite_transport_main
#include "native/src/sprite_can_shadow.cpp"
#undef main
#include <chrono>

int main(int argc,char**argv) {
  try {
    if(argc!=3)throw std::runtime_error("Expected a recorded run and output payload file");
    const std::string dir=argv[1];
    auto motors=load_motors(dir+"/motors.tsv");
    const auto kin=load_kinematics(dir+"/kinematics.tsv",motors);
    const auto limits=load_joint_safety(dir+"/joints.tsv");
    use_peak_pd_command_caps(motors);
    std::ifstream inputs(dir+"/native.bin.inputs",std::ios::binary),records(dir+"/native.bin",std::ios::binary);
    std::ofstream out(argv[2],std::ios::binary);
    std::array<double,71> input{};
    std::array<float,1024> floats{};
    std::array<double,62> action_target{};
    std::size_t frames=0,checks=0,stale=0;
    double max_delta=0.;
    const auto start=std::chrono::steady_clock::now();
    while(inputs.read(reinterpret_cast<char*>(input.data()),sizeof(input))) {
      if(!records.read(reinterpret_cast<char*>(floats.data()),sizeof(floats)) ||
         !records.read(reinterpret_cast<char*>(action_target.data()),sizeof(action_target)))
        throw std::runtime_error("Incomplete recorded target");
      IpcSocket ipc;ipc.has_target=true;
      for(std::size_t j=0;j<31;++j) {
        ipc.latest_target.position_rad[j]=std::clamp(action_target[31+j],limits[j].position_min,limits[j].position_max);
        ipc.latest_target.kp[j]=limits[j].kp_max;
        ipc.latest_target.kd[j]=limits[j].kd_max;
      }
      for(std::size_t m=0;m<31;++m) {
        motors[m].last_position=kin.motor_offset[m];motors[m].last_velocity=0.;
        for(std::size_t j=0;j<31;++j) {
          motors[m].last_position+=kin.joint_to_motor[m][j]*input[j];
          motors[m].last_velocity+=kin.joint_to_motor[m][j]*input[31+j];
        }
      }
      if(frames==0) {++frames;continue;}
      auto reference=motors;
      preview_final_motor_commands(reference,ipc,kin,false,true,true);
      std::array<std::array<std::uint8_t,8>,31> expected{};
      for(std::size_t m=0;m<31;++m)expected[m]=protected_policy_payload(reference[m],false,true);
      // A nonzero cached phase intentionally reproduces the former stale-target case.
      if(frames>1) {
        auto old=motors;ipc.motor_preview_count=1;
        preview_final_motor_commands(old,ipc,kin,false,true,false);
        for(std::size_t m=0;m<27;++m) {
          if(old[m].name.find("ankle_motor")!=std::string::npos)continue;
          if(std::abs(old[m].preview_position-reference[m].preview_position)>1e-12)++stale;
          max_delta=std::max(max_delta,std::abs(old[m].preview_position-reference[m].preview_position));
        }
      }
      for(std::size_t phase=0;phase<10;++phase) {
        auto candidate=motors;ipc.motor_preview_count=phase;
        preview_final_motor_commands(candidate,ipc,kin,false,true,true);
        for(std::size_t m=0;m<31;++m) {
          const auto payload=protected_policy_payload(candidate[m],false,true);
          if(payload!=expected[m])throw std::runtime_error("Phase-dependent stale payload");
          if(phase==0)out.write(reinterpret_cast<const char*>(payload.data()),payload.size());
          ++checks;
        }
      }
      motors=reference;++frames;
    }
    if(!inputs.eof() || frames<2 || !out)throw std::runtime_error("Incomplete replay");
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::cout<<"{\"frames\":"<<frames<<",\"phase_motor_checks\":"<<checks
             <<",\"stale_cache_differences\":"<<stale<<",\"max_stale_target_delta_rad\":"<<max_delta
             <<",\"wall_s\":"<<seconds<<",\"hardware_tx\":0}"<<std::endl;
  } catch(const std::exception&e) {std::cerr<<e.what()<<std::endl;return 1;}
}
