// Offline or receive-only simulation IPC; no CAN/device/powered entry.
#if defined(SPRITE_LOOP_TEST) || defined(SPRITE_SHADOW_LOOP) || defined(SPRITE_POWERED_LOOP)
#error SuperMimic replay cannot be built as a live or powered loop
#endif
#include "policy_core.hpp"
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
#include "policy_state_ipc.hpp"
#include "policy_imu_client.hpp"
#include "policy_feedback.hpp"
#endif
#include <onnxruntime_cxx_api.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
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
  try {
    if(argc<4)throw std::runtime_error("Usage: sprite_supermimic_replay MODEL FIXTURE OUTPUT [--benchmark] [--threads N] [--warmup N]");
    bool paced=false;int threads=1,warmup=50;
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
    std::string state_socket,imu_socket,feedback_config;
#endif
    for(int i=4;i<argc;++i) {
      const std::string option=argv[i];
      if(option=="--benchmark")paced=true;
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
      else if((option=="--state-socket" || option=="--imu-socket" || option=="--feedback-config") && i+1<argc) {
        const std::string value=argv[++i];
        if(option=="--state-socket")state_socket=value;
        else if(option=="--imu-socket")imu_socket=value;
        else feedback_config=value;
      }
#endif
      else if((option=="--threads" || option=="--warmup") && i+1<argc) {
        std::size_t used=0;const std::string value=argv[++i];const int n=std::stoi(value,&used);
        if(used!=value.size())throw std::runtime_error("invalid numeric option");
        if(option=="--threads")threads=n;else warmup=n;
      } else throw std::runtime_error("unknown or incomplete option");
    }
    if(threads<1 || threads>8 || warmup<0 || warmup>1000)throw std::runtime_error("option out of bounds");
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
    if(state_socket.empty() || imu_socket.empty() || feedback_config.empty() || paced)
      throw std::runtime_error("read-only IPC requires state/IMU/config and producer pacing");
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
    if(count==0 || count>10000 || core.steps>100000)throw std::runtime_error("fixture count/handoff out of bounds");
    read(in,core.defaults);read(in,core.offset);read(in,core.scale);
    for(int j=0;j<31;++j)if(!std::isfinite(core.defaults[j]) || !std::isfinite(core.offset[j]) ||
       !std::isfinite(core.scale[j]) || core.scale[j]<=1e-12)throw std::runtime_error("invalid affine configuration");
    std::vector<Input> inputs(count);
    for(auto& row:inputs) {
      read(in,row);
      for(double v:row)if(!std::isfinite(v))throw std::runtime_error("nonfinite recorded state");
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
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
    std::ifstream feedback_file(feedback_config,std::ios::binary);std::array<char,8> sensor_magic{};
    read(feedback_file,sensor_magic);
    if(std::memcmp(sensor_magic.data(),"SPMSEN01",8))throw std::runtime_error("feedback configuration magic mismatch");
    std::uint64_t motor_hash=0;read(feedback_file,motor_hash);sprite_policy::FeedbackMap feedback;
    read(feedback_file,feedback.motor_zero);read(feedback_file,feedback.motor_to_joint);
    if(feedback_file.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing feedback configuration");
    for(double v:feedback.motor_zero)if(!std::isfinite(v))throw std::runtime_error("nonfinite feedback zero");
    for(double v:feedback.motor_to_joint)if(!std::isfinite(v))throw std::runtime_error("nonfinite feedback matrix");
    sprite_policy::NativeImuClient imu(imu_socket,true);
    sprite_policy::ImuSnapshot sample;const auto wait_start=Clock::now();
    while(!imu.snapshot(sample)) {
      if(Clock::now()-wait_start>std::chrono::seconds(1))throw std::runtime_error("IMU startup timeout");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    sprite_policy::PolicyStateChannel channel(state_socket,motor_hash,0,30000000,1000);
    channel.receive_only(); // Kernel-enforced EOF on actor->state-producer direction.
    std::vector<double> state_age,imu_age;
#endif
    // All input is resident; outputs and logs are written only after the timed loop.
    std::vector<Record> records(count);std::vector<Timing> timings(count);
    const auto anchor=Clock::now();unsigned deadline_misses=0;
    for(std::size_t i=0;i<count;++i) {
      const auto deadline=anchor+std::chrono::milliseconds(20*i);
      if(paced)std::this_thread::sleep_until(deadline);
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
      const auto state=channel.receive(sprite_policy::monotonic_ns);
      if(!imu.snapshot(sample))throw std::runtime_error("IMU missing");
      const auto received=sprite_policy::monotonic_ns();
      if(sample.start>received || received-sample.start>30000000)throw std::runtime_error("IMU stale before inference");
      std::array<double,69> sensor{};
      std::copy(state.position.begin(),state.position.end(),sensor.begin());
      std::copy(state.velocity.begin(),state.velocity.end(),sensor.begin()+31);
      std::copy_n(sample.values.begin()+3,3,sensor.begin()+62);
      std::copy_n(sample.values.begin()+6,4,sensor.begin()+65);
      const auto body=feedback.convert(sensor);std::copy(body.begin(),body.end(),inputs[i].begin());
#endif
      const auto start=Clock::now();auto& record=records[i];
      record.obs=core.observe(inputs[i]);const auto before_infer=Clock::now();
      infer(record.obs,record.raw);const auto after_infer=Clock::now();
      record.target=core.latch(record.raw,inputs[i]);record.action=core.previous;
      const auto finish=Clock::now();
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
      const auto done=sprite_policy::monotonic_ns();
      sprite_policy::state_fresh(state,done,30000000);
      if(done<sample.start || done-sample.start>30000000)throw std::runtime_error("IMU stale after inference");
      state_age.push_back((received-state.stamp)/1e6);imu_age.push_back((received-sample.start)/1e6);
      timings[i]={ms(after_infer-before_infer),ms(finish-start),(received-state.stamp)/1e6,(done-state.stamp)/1e6};
      if(done-state.stamp>20000000)++deadline_misses;
#else
      timings[i]={ms(after_infer-before_infer),ms(finish-start),paced?ms(start-deadline):0.,paced?ms(finish-deadline):0.};
      if(paced && finish>deadline+std::chrono::milliseconds(20))++deadline_misses;
#endif
    }
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
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
    std::ofstream consumed(std::string(argv[3])+".inputs",std::ios::binary);
    for(const auto& row:inputs)write(consumed,row);
    consumed.close();if(!consumed)throw std::runtime_error("consumed state write failed");
#endif
    std::ostringstream report;report<<std::setprecision(12);
    report<<"{\"profile\":\"supermimic_v5_10f\",\"mode\":\""
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
          <<"receive_only_simulation_state_ipc"
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
#ifdef SPRITE_SUPERMIMIC_IPC_READONLY
    report<<",\"state_age_ms\":";stats(report,state_age);
    report<<",\"imu_age_ms\":";stats(report,imu_age);
    report<<",\"state_socket_transmitted_bytes\":0";
#endif
    report<<",\"deadline_misses\":"<<deadline_misses<<",\"hardware_tx\":0,\"live_devices_opened\":0}";
    std::ofstream json(std::string(argv[3])+".timing.json");json<<report.str()<<'\n';json.close();
    if(!json)throw std::runtime_error("timing report write failed");
    std::cout<<report.str()<<'\n';return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
