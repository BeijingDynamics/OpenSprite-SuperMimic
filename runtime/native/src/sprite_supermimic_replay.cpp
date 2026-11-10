// Offline, receive-only or protected target IPC. Only the controller owns CAN.
#if defined(SPRITE_LOOP_TEST) || defined(SPRITE_SHADOW_LOOP) || defined(SPRITE_POWERED_LOOP)
#error SuperMimic replay cannot be built as a live or powered loop
#endif
#if defined(SPRITE_SUPERMIMIC_IPC_READONLY) && defined(SPRITE_SUPERMIMIC_PROTECTED_IPC)
#error SuperMimic IPC modes must use separate executables
#endif
#if defined(SPRITE_SUPERMIMIC_IPC_READONLY) || defined(SPRITE_SUPERMIMIC_PROTECTED_IPC)
#define SPRITE_SUPERMIMIC_STATE_IPC
#endif
#include "policy_core.hpp"
#ifdef SPRITE_SUPERMIMIC_STATE_IPC
#include "policy_state_ipc.hpp"
#include "policy_imu_client.hpp"
#include "policy_feedback.hpp"
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
#include "sling_window.hpp"
#include <csignal>
#include <fcntl.h>
#endif
#endif
#include <onnxruntime_cxx_api.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock=std::chrono::steady_clock;
using Core=sprite_policy::SuperMimicCore;
using Input=std::array<double,71>;
struct Record {
  std::array<float,Core::input_size> obs{};
  std::array<float,31> raw{};
  sprite_policy::Joints action{},target{};
};
struct Timing { double inference=0,service=0,wake=0,finish=0; };
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
struct StartupSample {
  sprite_policy::ProtectionVector waypoint{};
  double alpha=0., tracking_error=0., goal_error=0.;
  unsigned stable=0;
};
struct ActorCycleTiming {
  std::uint64_t sequence=0, feedback_ns=0, receive_ns=0, infer_start_ns=0,
      infer_end_ns=0, target_ns=0, sent_ns=0;
};
struct WalkCheckpoint {
  std::uint64_t sequence=0;
  sprite_policy::ProtectionVector measured{}, goal{}, velocity{};
};
bool save_walk_checkpoints(const char* output,const std::array<WalkCheckpoint,2>& checkpoints) {
  if(!checkpoints[0].sequence)return true;
  std::ofstream trace(std::string(output)+".walk-checkpoints.csv");
  trace<<std::setprecision(17)<<"sequence,joint_index,measured_rad,goal_rad,velocity_rad_s\n";
  for(const auto& sample:checkpoints) {
    if(!sample.sequence)break;
    for(std::size_t j=0;j<sample.measured.size();++j)
      trace<<sample.sequence<<','<<j<<','<<sample.measured[j]<<','<<sample.goal[j]<<','<<sample.velocity[j]<<'\n';
  }
  trace.close();
  return static_cast<bool>(trace);
}
bool save_actor_cycles(const char* output,const std::vector<ActorCycleTiming>& cycles) {
  std::ofstream trace(std::string(output)+".ipc.csv");
  trace<<"sequence,feedback_ns,receive_ns,infer_start_ns,infer_end_ns,target_ns,sent_ns\n";
  for(const auto& t:cycles) {
    if(!t.sequence)break;
    trace<<t.sequence<<','<<t.feedback_ns<<','<<t.receive_ns<<','<<t.infer_start_ns<<','
         <<t.infer_end_ns<<','<<t.target_ns<<','<<t.sent_ns<<'\n';
  }
  trace.close();
  return static_cast<bool>(trace);
}
#endif
template<class T> void read(std::istream& stream,T& value) {
  if(!stream.read(reinterpret_cast<char*>(&value),sizeof(value)))
    throw std::runtime_error("truncated replay fixture");
}
template<class T> void write(std::ostream& stream,const T& value) {
  if(!stream.write(reinterpret_cast<const char*>(&value),sizeof(value)))
    throw std::runtime_error("replay output write failed");
}
double ms(Clock::duration value) {
  return std::chrono::duration<double,std::milli>(value).count();
}
void stats(std::ostream& stream,const std::vector<double>& values) {
  auto sorted=values;std::sort(sorted.begin(),sorted.end());
  double sum=0;for(double v:values)sum+=v;
  const auto percentile=[&](double f){return sorted[static_cast<std::size_t>(std::ceil(f*sorted.size()))-1];};
  stream<<"{\"mean\":"<<sum/values.size()<<",\"p50\":"<<percentile(.50)
        <<",\"p95\":"<<percentile(.95)<<",\"p99\":"<<percentile(.99)
        <<",\"max\":"<<sorted.back()<<'}';
}
}

int main(int argc,char** argv) {
  std::vector<Input> inputs;
  std::vector<Record> records;
  std::vector<Timing> timings;
  std::size_t completed_frames=0;
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
  std::vector<ActorCycleTiming> cycles;
  std::array<WalkCheckpoint,2> walk_checkpoints{};
  std::vector<StartupSample> startup_trace;
#endif
  try {
    if(argc<4)throw std::runtime_error("Usage: sprite_supermimic_replay MODEL FIXTURE OUTPUT [--benchmark] [--threads N] [--warmup N]");
    bool paced=false;int threads=1,warmup=50;
#ifdef SPRITE_SUPERMIMIC_STATE_IPC
    std::string state_socket,imu_socket,feedback_config,imu_kind="software";
#endif
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
    std::string target_config;
    bool tracked_startup=false;
    bool xbox_full_leg_waist_gains=false;
    bool training_full_body=false;
    bool direct_policy_start=false;
    double walk_speed=0.;
    int sling_ready_fd=-1;
#endif
    for(int i=4;i<argc;++i) {
      const std::string option=argv[i];
      if(option=="--benchmark")paced=true;
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      else if(option=="--target-config" && i+1<argc)target_config=argv[++i];
      else if(option=="--tracked-startup")tracked_startup=true;
      else if(option=="--xbox-full-leg-waist-gains")xbox_full_leg_waist_gains=true;
      else if(option=="--training-full-body")training_full_body=true;
      else if(option=="--direct-policy-start")direct_policy_start=true;
      else if(option=="--walk-020" || option=="--walk-025" || option=="--walk-030") {
        if(walk_speed>0.)throw std::runtime_error("only one walking profile can be selected");
        walk_speed=option=="--walk-020"?.20:option=="--walk-025"?.25:.30;
      }
      else if(option=="--sling-ready-fd" && i+1<argc) {
        std::size_t used=0;
        const std::string value=argv[++i];
        sling_ready_fd=std::stoi(value,&used);
        if(used!=value.size() || sling_ready_fd<3)
          throw std::runtime_error("invalid sling readiness descriptor");
      }
#endif
#ifdef SPRITE_SUPERMIMIC_STATE_IPC
      else if((option=="--state-socket" || option=="--imu-socket" || option=="--feedback-config" || option=="--imu-kind") && i+1<argc) {
        const std::string value=argv[++i];
        if(option=="--state-socket")state_socket=value;
        else if(option=="--imu-socket")imu_socket=value;
        else if(option=="--feedback-config")feedback_config=value;
        else imu_kind=value;
      }
#endif
      else if((option=="--threads" || option=="--warmup") && i+1<argc) {
        std::size_t used=0;const std::string value=argv[++i];const int n=std::stoi(value,&used);
        if(used!=value.size())throw std::runtime_error("invalid numeric option");
        if(option=="--threads")threads=n;else warmup=n;
      } else throw std::runtime_error("unknown or incomplete option");
    }
    if(threads<1 || threads>8 || warmup<0 || warmup>1000)throw std::runtime_error("option out of bounds");
#ifdef SPRITE_SUPERMIMIC_STATE_IPC
    if(state_socket.empty() || imu_socket.empty() || feedback_config.empty() || paced)
      throw std::runtime_error("state IPC requires state/IMU/config and producer pacing");
    if(imu_kind!="software" && imu_kind!="icm")throw std::runtime_error("IMU kind must be software or icm");
#endif
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
    if(target_config.empty() || imu_kind!="icm")throw std::runtime_error("protected IPC requires ICM and target configuration");
    if(xbox_full_leg_waist_gains && !tracked_startup)
      throw std::runtime_error("full Xbox leg and waist gains require tracked startup");
    if(training_full_body && (!tracked_startup || xbox_full_leg_waist_gains))
      throw std::runtime_error("training full-body mode requires tracked startup without Xbox overlay");
    if(direct_policy_start && !training_full_body)
      throw std::runtime_error("direct policy start requires training full-body mode");
    if(walk_speed>0. && (!(xbox_full_leg_waist_gains || training_full_body) || sling_ready_fd<0))
      throw std::runtime_error("walking requires full gains and sling readiness");
#endif
    const std::uint32_t endian=1;
    if(*reinterpret_cast<const char*>(&endian)!=1)throw std::runtime_error("little endian required");
    std::ifstream in(argv[2],std::ios::binary);
    if(!in)throw std::runtime_error("cannot open fixture");
    std::array<char,8> magic{};read(in,magic);
    if(std::memcmp(magic.data(),"SPMIM001",8))throw std::runtime_error("SuperMimic profile magic mismatch");
    std::array<std::uint32_t,6> header{};read(in,header);
    if(header[0]!=10 || header[1]!=993 || header[2]!=31 || header[3]!=50)
      throw std::runtime_error("SuperMimic fixture profile dimensions/rate mismatch");
    const auto count=header[4];Core core;core.steps=header[5];
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
    std::uint64_t body_stable_frames=0;
    if(sling_ready_fd>=0) {
      const int flags=fcntl(sling_ready_fd,F_GETFL);
      const auto duration=walk_speed>0.?sprite_policy::kWalkDurationSeconds:sprite_policy::kSlingDurationSeconds;
      if(!tracked_startup || count!=duration*50+1 ||
         flags<0 || !(flags&O_NONBLOCK) || (flags&O_ACCMODE)!=O_WRONLY)
        throw std::runtime_error("invalid protected sling actor profile");
      std::signal(SIGPIPE,SIG_IGN);
    }
#endif
    if(count==0 || count>10000 || core.steps>100000)throw std::runtime_error("fixture count/handoff out of bounds");
    read(in,core.defaults);read(in,core.offset);read(in,core.scale);
    for(int j=0;j<31;++j)if(!std::isfinite(core.defaults[j]) || !std::isfinite(core.offset[j]) ||
       !std::isfinite(core.scale[j]) || core.scale[j]<=1e-12)throw std::runtime_error("invalid affine configuration");
    inputs.resize(count);
    for(auto& row:inputs) {
      read(in,row);
      for(double v:row)if(!std::isfinite(v))throw std::runtime_error("nonfinite recorded state");
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      if(row[68]!=0 || row[69]!=0 || row[70]!=0)throw std::runtime_error("commissioning requires zero velocity command");
#endif
    }
    if(in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing fixture bytes");
    const auto load_start=Clock::now();
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"sprite-supermimic-file-only");
    Ort::SessionOptions options;options.SetIntraOpNumThreads(threads);options.SetInterOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    Ort::Session session(env,argv[1],options);
    if(session.GetInputCount()!=1 || session.GetOutputCount()!=1)throw std::runtime_error("model IO count mismatch");
    const auto input_type=session.GetInputTypeInfo(0),output_type=session.GetOutputTypeInfo(0);
    const auto input_info=input_type.GetTensorTypeAndShapeInfo(),output_info=output_type.GetTensorTypeAndShapeInfo();
    const auto is_shape=[](const std::vector<int64_t>& shape,int width) {
      return shape.size()==2 && (shape[0]==1 || shape[0]==-1) && shape[1]==width;
    };
    if(input_info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
       output_info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
       !is_shape(input_info.GetShape(),993) || !is_shape(output_info.GetShape(),31))
      throw std::runtime_error("SuperMimic model shape/type mismatch (requires 993 -> 31 float32)");
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_name=session.GetInputNameAllocated(0,allocator),output_name=session.GetOutputNameAllocated(0,allocator);
    const char* ins[]={input_name.get()};const char* outs[]={output_name.get()};
    const std::array<int64_t,2> shape{1,993};
    auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
    const double load_ms=ms(Clock::now()-load_start);
    const auto infer=[&](std::array<float,993>& observation,std::array<float,31>& raw) {
      auto tensor=Ort::Value::CreateTensor<float>(memory,observation.data(),observation.size(),shape.data(),shape.size());
      auto values=session.Run(Ort::RunOptions{nullptr},ins,&tensor,1,outs,1);
      if(values.size()!=1 || values[0].GetTensorTypeAndShapeInfo().GetShape()!=std::vector<int64_t>({1,31}))
        throw std::runtime_error("invalid runtime actor output shape");
      std::copy_n(values[0].GetTensorData<float>(),31,raw.begin());
    };
    auto warm_core=core;
    for(int i=0;i<warmup;++i) {
      const auto& row=inputs[static_cast<std::size_t>(i)%inputs.size()];
      auto obs=warm_core.observe(row);std::array<float,31> raw{};infer(obs,raw);warm_core.latch(raw,row);
    }
#ifdef SPRITE_SUPERMIMIC_STATE_IPC
    std::ifstream feedback_file(feedback_config,std::ios::binary);std::array<char,8> sensor_magic{};
    read(feedback_file,sensor_magic);
    if(std::memcmp(sensor_magic.data(),"SPMSEN01",8))throw std::runtime_error("feedback configuration magic mismatch");
    std::uint64_t motor_hash=0;read(feedback_file,motor_hash);sprite_policy::FeedbackMap feedback;
    read(feedback_file,feedback.motor_zero);read(feedback_file,feedback.motor_to_joint);
    if(feedback_file.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing feedback configuration");
    for(double v:feedback.motor_zero)if(!std::isfinite(v))throw std::runtime_error("nonfinite feedback zero");
    for(double v:feedback.motor_to_joint)if(!std::isfinite(v))throw std::runtime_error("nonfinite feedback matrix");
    sprite_policy::NativeImuClient imu(imu_socket,true,imu_kind=="icm");
    sprite_policy::ImuSnapshot sample;const auto wait_start=Clock::now();
    while(!imu.snapshot(sample)) {
      if(Clock::now()-wait_start>std::chrono::seconds(1))throw std::runtime_error("IMU startup timeout");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
    std::ifstream target_file(target_config,std::ios::binary);
    std::array<char,8> target_magic{};read(target_file,target_magic);
    if(std::memcmp(target_magic.data(),tracked_startup?"SPMTGT02":"SPMTGT01",8))
      throw std::runtime_error("target configuration magic mismatch");
    std::uint64_t joint_hash=0;read(target_file,joint_hash);
    sprite_policy::ImpedanceTarget impedance;
    sprite_policy::ProtectionVector lower{},upper{};
    read(target_file,lower);read(target_file,upper);read(target_file,impedance.kp);read(target_file,impedance.kd);
    std::optional<sprite_policy::TrackedStartup> tracked;
    if(tracked_startup) {
      std::array<sprite_policy::StartupJoint,31> roles{};
      read(target_file,roles);
      tracked.emplace(roles, sling_ready_fd>=0 ? sprite_policy::kSlingStartupHoldSeconds : 1., training_full_body, direct_policy_start);
      if(xbox_full_leg_waist_gains) {
        for(std::size_t j=0;j<roles.size();++j) {
          using Joint=sprite_policy::StartupJoint;
          const double max_kd=(roles[j]==Joint::arm || roles[j]==Joint::hold)?3.:5.;
          if(impedance.kd[j]>max_kd)
            throw std::runtime_error("full Xbox Kd exceeds joint group limit");
        }
      }
    }
    if(!joint_hash || target_file.peek()!=std::char_traits<char>::eof())
      throw std::runtime_error("invalid target configuration metadata");
    sprite_policy::validate_target(impedance);
    sprite_policy::ProtectionVector ones{};ones.fill(1.);
    sprite_policy::TargetProjector limits(lower,upper,ones,ones,1.,(xbox_full_leg_waist_gains || training_full_body)?5.:3.);
    sprite_policy::StartupRamp startup(50,1.,4.);
    double peak_target_kp=0.;
    sprite_policy::PolicyStateChannel channel(state_socket,motor_hash,joint_hash,20000000,1000);
#else
    sprite_policy::PolicyStateChannel channel(state_socket,motor_hash,0,30000000,1000);
    channel.receive_only(); // Kernel-enforced EOF on actor->state-producer direction.
#endif
    std::vector<double> state_age,imu_age;
#endif
    // All input is resident; outputs and logs are written only after the timed loop.
    records.resize(count);timings.resize(count);
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
    startup_trace.resize(tracked_startup?count:0);
    cycles.resize(count);
#endif
    const auto anchor=Clock::now();unsigned deadline_misses=0;
    for(std::size_t i=0;i<count;++i) {
      const auto deadline=anchor+std::chrono::milliseconds(20*i);
      if(paced)std::this_thread::sleep_until(deadline);
#ifdef SPRITE_SUPERMIMIC_STATE_IPC
      const auto state=channel.receive(sprite_policy::monotonic_ns);
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      auto& cycle=cycles[i];
      cycle={state.sequence,state.stamp,sprite_policy::monotonic_ns()};
#endif
      if(!imu.snapshot(sample))throw std::runtime_error("IMU missing");
      const auto received=sprite_policy::monotonic_ns();
      if(sample.start>received || received-sample.start>30000000)throw std::runtime_error("IMU stale before inference");
      std::array<double,69> sensor{};
      std::copy(state.position.begin(),state.position.end(),sensor.begin());
      std::copy(state.velocity.begin(),state.velocity.end(),sensor.begin()+31);
      std::copy_n(sample.values.begin()+3,3,sensor.begin()+62);
      std::copy_n(sample.values.begin()+6,4,sensor.begin()+65);
      const auto body=feedback.convert(sensor);std::copy(body.begin(),body.end(),inputs[i].begin());
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      if(walk_speed>0.) {
        if(state.sequence!=i+1)throw std::runtime_error("walking state sequence is not contiguous");
        inputs[i][68]=sprite_policy::walk_vx(state.sequence,walk_speed);
      }
#endif
#endif
      const auto start=Clock::now();auto& record=records[i];
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      const bool handshake_only=direct_policy_start && i==0;
      if(direct_policy_start && i==1) { core.reset_episode(); core.steps=0; }
#endif
      record.obs=core.observe(inputs[i]);const auto before_infer=Clock::now();
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      cycle.infer_start_ns=sprite_policy::monotonic_ns();
#endif
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      if(!handshake_only) infer(record.obs,record.raw);
#else
      infer(record.obs,record.raw);
#endif
      const auto after_infer=Clock::now();
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      cycle.infer_end_ns=sprite_policy::monotonic_ns();
#endif
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      if(handshake_only) std::copy_n(inputs[i].begin(),31,record.target.begin());
      else record.target=core.latch(record.raw,inputs[i],training_full_body);
#else
      record.target=core.latch(record.raw,inputs[i]);
#endif
      record.action=core.previous;
      const auto finish=Clock::now();
#ifdef SPRITE_SUPERMIMIC_STATE_IPC
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      if(inputs[i][67]>-.5)throw std::runtime_error("commissioning body tilt limit");
      if(std::any_of(record.raw.begin(),record.raw.end(),[](float x){return std::abs(x)>20.;}))
        throw std::runtime_error("SuperMimic raw action outside commissioning bounds");
      impedance.position=record.target;
      sprite_policy::ProtectionVector measured{}, velocity{};
      std::copy_n(inputs[i].begin(),31,measured.begin());
      std::copy_n(inputs[i].begin()+31,31,velocity.begin());
      if(!(tracked?tracked->captured():startup.captured()))(void)limits.require_position(measured);
      const auto goal=limits.project(impedance);
      auto target=goal;
      if(handshake_only) {
        // Transport readiness only: no inference, no action history, no control episode.
        target={}; target.position=measured;
      } else target=tracked?tracked->apply(goal,measured,velocity):startup.apply(goal,measured);
#endif
      const auto done=sprite_policy::monotonic_ns();
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      cycle.target_ns=done;
#endif
      sprite_policy::state_fresh(state,done,30000000);
      if(done<sample.start || done-sample.start>30000000)throw std::runtime_error("IMU stale after inference");
      state_age.push_back((received-state.stamp)/1e6);imu_age.push_back((received-sample.start)/1e6);
      timings[i]={ms(after_infer-before_infer),ms(finish-start),(received-state.stamp)/1e6,(done-state.stamp)/1e6};
      if(done-state.stamp>20000000)++deadline_misses;
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
      if(deadline_misses)throw std::runtime_error("SuperMimic oldest-feedback 20 ms deadline exceeded before send");
      channel.send(target,state,done);
      cycle.sent_ns=sprite_policy::monotonic_ns();
      if(tracked) {
        // Full-body SuperMimic matches training a_prev.copy_(a). Measured q/qd
        // remain actual feedback even if downstream PD protection clips a target.
        // Preserve the older experimental mode for explicit historical replay.
        if(!training_full_body)
          for(std::size_t j=0;j<target.position.size();++j)
            core.previous[j]=(target.position[j]-core.offset[j])/core.scale[j];
        startup_trace[i]={target.position,tracked->alpha(),tracked->tracking_error_rad,
                         tracked->goal_error_rad,tracked->stable_frames()};
        const auto last_checkpoint=walk_speed>0.?sprite_policy::kWalkReadySequence:sprite_policy::kSlingReadySequence;
        if(sling_ready_fd>=0 && state.sequence<=last_checkpoint) {
          constexpr double cos_ten_degrees=.984807753012208;
          const bool quiet=std::hypot(inputs[i][62],inputs[i][63],inputs[i][64])<=.15 &&
                           inputs[i][67]<=-cos_ten_degrees;
          body_stable_frames=quiet?std::min(body_stable_frames+1,sprite_policy::kSlingStableFrames):0;
          if(state.sequence==sprite_policy::kSlingReadySequence || state.sequence==last_checkpoint) {
            const auto index=state.sequence==sprite_policy::kSlingReadySequence?0:1;
            walk_checkpoints[index]={state.sequence,measured,goal.position,velocity};
            const sprite_policy::SlingReadiness ready{state.sequence,tracked->stable_frames(),
                                                     body_stable_frames};
            if(write(sling_ready_fd,&ready,sizeof(ready))!=static_cast<ssize_t>(sizeof(ready)))
              throw std::runtime_error("cannot report sling readiness");
          }
        }
      }
      peak_target_kp=std::max(peak_target_kp,*std::max_element(target.kp.begin(),target.kp.end()));
#endif
#else
      timings[i]={ms(after_infer-before_infer),ms(finish-start),paced?ms(start-deadline):0.,paced?ms(finish-deadline):0.};
      if(paced && finish>deadline+std::chrono::milliseconds(20))++deadline_misses;
#endif
      completed_frames=i+1;
    }
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
    channel.await_controller_close(2000);
    if(!save_actor_cycles(argv[3],cycles))throw std::runtime_error("actor IPC timing trace write failed");
    if(!save_walk_checkpoints(argv[3],walk_checkpoints))
      throw std::runtime_error("walking checkpoint trace write failed");
#endif
    const double wall_ms=ms(Clock::now()-anchor);
    std::ofstream out(argv[3],std::ios::binary),csv(std::string(argv[3])+".timing.csv");
    if(!out || !csv)throw std::runtime_error("cannot open output files");
    csv<<std::setprecision(12)<<"frame,inference_ms,service_ms,wake_lateness_ms,finish_lateness_ms\n";
    std::vector<double> inference,service,wake,finish;
    for(std::size_t i=0;i<count;++i) {
      const auto& r=records[i];write(out,r.obs);write(out,r.raw);write(out,r.action);write(out,r.target);
      const auto& t=timings[i];
      csv<<i<<','<<t.inference<<','<<t.service<<','<<t.wake<<','<<t.finish<<'\n';
      inference.push_back(t.inference);service.push_back(t.service);wake.push_back(t.wake);finish.push_back(t.finish);
    }
    out.close();csv.close();if(!out || !csv)throw std::runtime_error("output flush failed");
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
    if(tracked_startup) {
      std::ofstream trace(std::string(argv[3])+".startup.csv");
      trace<<std::setprecision(12)<<"frame,gain_alpha,tracking_error_rad,goal_error_rad,stable_frames";
      for(std::size_t j=0;j<31;++j)trace<<",waypoint_"<<j<<"_rad";
      trace<<'\n';
      for(std::size_t i=0;i<startup_trace.size();++i) {
        const auto& s=startup_trace[i];
        trace<<i<<','<<s.alpha<<','<<s.tracking_error<<','<<s.goal_error<<','<<s.stable;
        for(double q:s.waypoint)trace<<','<<q;
        trace<<'\n';
      }
      trace.close();if(!trace)throw std::runtime_error("startup trace write failed");
    }
#endif
#ifdef SPRITE_SUPERMIMIC_STATE_IPC
    std::ofstream consumed(std::string(argv[3])+".inputs",std::ios::binary);
    for(const auto& row:inputs)write(consumed,row);
    consumed.close();if(!consumed)throw std::runtime_error("consumed state write failed");
#endif
    std::ostringstream report;report<<std::setprecision(12);
    report<<"{\"profile\":\"supermimic_v5_10f\",\"mode\":\""
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
          <<"protected_target_ipc"
#elif defined(SPRITE_SUPERMIMIC_IPC_READONLY)
          <<"receive_only_state_ipc"
#else
          <<(paced?"paced_recorded_states_no_hardware":"offline_replay_no_hardware")
#endif
          <<"\",\"frames\":"<<count<<",\"policy_hz\":50,\"threads\":"<<threads
          <<",\"warmup_frames\":"<<warmup<<",\"session_load_ms\":"<<load_ms
          <<",\"loop_wall_ms\":"<<wall_ms<<",\"ort_version\":\""<<OrtGetApiBase()->GetVersionString()
          <<"\",\"inference_ms\":";stats(report,inference);
    report<<",\"service_ms\":";stats(report,service);
    report<<",\"wake_lateness_ms\":";stats(report,wake);
    report<<",\"finish_lateness_ms\":";stats(report,finish);
#ifdef SPRITE_SUPERMIMIC_STATE_IPC
    report<<",\"state_age_ms\":";stats(report,state_age);
    report<<",\"imu_age_ms\":";stats(report,imu_age);
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
    std::uint64_t projected_targets=0;
    for(auto count:limits.clamp_count)projected_targets+=count;
    report<<",\"state_socket_transmitted_bytes\":"<<1280*count<<",\"maximum_target_kp\":"<<peak_target_kp
          <<",\"gain_ramp_completed\":"<<((tracked?tracked->complete():startup.complete())?"true":"false")
          <<",\"final_gain_ramp_alpha\":"<<(tracked?tracked->alpha():startup.last_alpha)
          <<",\"tracked_startup\":"<<(tracked_startup?"true":"false")
          <<",\"xbox_full_leg_waist_gains\":"<<(xbox_full_leg_waist_gains?"true":"false")
          <<",\"training_full_body\":"<<(training_full_body?"true":"false")
          <<",\"direct_policy_start\":"<<(direct_policy_start?"true":"false")
          <<",\"formal_policy_frames\":"<<(direct_policy_start?count-1:count)
          <<",\"action_history\":\""<<(training_full_body?"policy_output":"admitted_waypoint")<<"\""
          <<",\"walk_speed_m_s\":"<<walk_speed
          <<",\"startup_final_tracking_error_rad\":"<<(tracked?tracked->tracking_error_rad:0.)
          <<",\"startup_final_goal_error_rad\":"<<(tracked?tracked->goal_error_rad:0.)
          <<",\"startup_stable_frames\":"<<(tracked?tracked->stable_frames():0)
          <<",\"startup_settled\":"<<((tracked && tracked->settled())?"true":"false")
          <<",\"projected_target_joint_frames\":"<<projected_targets
          <<",\"maximum_joint_target_overshoot_rad\":"<<limits.maximum_overshoot;
#else
    report<<",\"state_socket_transmitted_bytes\":0";
#endif
    report<<",\"imu_kind\":\""<<imu_kind<<"\"";
#endif
    report<<",\"deadline_misses\":"<<deadline_misses<<",\"hardware_tx\":0,\"live_devices_opened\":0}";
    std::ofstream json(std::string(argv[3])+".timing.json");json<<report.str()<<'\n';json.close();
    if(!json)throw std::runtime_error("timing report write failed");
    std::cout<<report.str()<<'\n';return 0;
  } catch(const std::exception& error) {
    // Preserve only completed iterations. Unconsumed fixture rows and partially
    // computed actions must never masquerade as executed feedback or targets.
    // Flush after leaving the timed loop, including controller-initiated stops.
    if(argc>=4 && completed_frames) {
      try {
        std::ofstream out(argv[3],std::ios::binary);
        std::ofstream consumed(std::string(argv[3])+".inputs",std::ios::binary);
        std::ofstream timing(std::string(argv[3])+".timing.csv");
        timing<<std::setprecision(12)<<"frame,inference_ms,service_ms,wake_lateness_ms,finish_lateness_ms\n";
        for(std::size_t i=0;i<completed_frames;++i) {
          const auto& r=records[i];write(out,r.obs);write(out,r.raw);write(out,r.action);write(out,r.target);
          write(consumed,inputs[i]);
          const auto& t=timings[i];
          timing<<i<<','<<t.inference<<','<<t.service<<','<<t.wake<<','<<t.finish<<'\n';
        }
        out.close();consumed.close();timing.close();
        if(!out || !consumed || !timing)throw std::runtime_error("partial frame flush failed");
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
        if(!startup_trace.empty()) {
          std::ofstream trace(std::string(argv[3])+".startup.csv");
          trace<<std::setprecision(12)<<"frame,gain_alpha,tracking_error_rad,goal_error_rad,stable_frames";
          for(std::size_t j=0;j<31;++j)trace<<",waypoint_"<<j<<"_rad";
          trace<<'\n';
          for(std::size_t i=0;i<completed_frames;++i) {
            const auto& s=startup_trace[i];
            trace<<i<<','<<s.alpha<<','<<s.tracking_error<<','<<s.goal_error<<','<<s.stable;
            for(double q:s.waypoint)trace<<','<<q;
            trace<<'\n';
          }
          trace.close();if(!trace)throw std::runtime_error("partial startup flush failed");
        }
#endif
        std::ofstream metadata(std::string(argv[3])+".partial.json");
        metadata<<"{\"complete\":false,\"completed_frames\":"<<completed_frames<<"}\n";
        metadata.close();if(!metadata)throw std::runtime_error("partial trace metadata flush failed");
      } catch(const std::exception& flush_error) {
        std::cerr<<"partial trace save failed: "<<flush_error.what()<<'\n';
      }
    }
#ifdef SPRITE_SUPERMIMIC_PROTECTED_IPC
    if(!cycles.empty() && !save_actor_cycles(argv[3],cycles))
      std::cerr<<"partial actor IPC timing trace write failed\n";
    if(argc>=4 && !save_walk_checkpoints(argv[3],walk_checkpoints))
      std::cerr<<"partial walking checkpoint trace write failed\n";
#endif
    std::cerr<<error.what()<<'\n';return 1;
  }
}
