// Isolated recorded-observation CPU inference probe. No device or IPC access.
#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

using Clock = std::chrono::steady_clock;
struct Record {
  std::array<float,993> obs;
  std::array<float,31> raw;
  std::array<double,31> action, target;
};
static_assert(sizeof(Record)==4592);
long long ns(clockid_t id=CLOCK_MONOTONIC) {
  timespec t{};
  if(clock_gettime(id,&t))throw std::runtime_error("clock_gettime failed");
  return t.tv_sec*1000000000LL+t.tv_nsec;
}
std::vector<int> tids() {
  std::vector<int> out;
  for(const auto& e:std::filesystem::directory_iterator("/proc/self/task"))
    out.push_back(std::stoi(e.path().filename()));
  std::sort(out.begin(),out.end());return out;
}
void pin(int tid, std::initializer_list<int> cpus) {
  cpu_set_t mask;CPU_ZERO(&mask);for(int c:cpus)CPU_SET(c,&mask);
  if(sched_setaffinity(tid,sizeof(mask),&mask))throw std::runtime_error("affinity failed");
}
std::string affinity(int tid) {
  cpu_set_t mask;CPU_ZERO(&mask);
  if(sched_getaffinity(tid,sizeof(mask),&mask))throw std::runtime_error("read affinity failed");
  std::string s;
  for(int i=0;i<CPU_SETSIZE;++i)if(CPU_ISSET(i,&mask))s+=(s.empty()?"":",")+std::to_string(i);
  return s;
}
struct Sched {long long run=0, wait=0, slices=0;};
Sched scheduler(int tid) {
  Sched s;
  std::ifstream f("/proc/self/task/"+std::to_string(tid)+"/schedstat");
  if(!(f>>s.run>>s.wait>>s.slices))throw std::runtime_error("schedstat unavailable");
  return s;
}
long long freq(int cpu) {
  long long x=-1;
  std::ifstream("/sys/devices/system/cpu/cpu"+std::to_string(cpu)+"/cpufreq/scaling_cur_freq")>>x;
  return x;
}
struct Row {
  int frame=0,input=0,cpu_before=0,cpu_after=0;
  long long start=0,end=0,process_cpu=0,main_cpu=0,voluntary=0,involuntary=0;
  long long freq4_before=0,freq5_before=0,freq4_after=0,freq5_after=0;
  Sched main_delta, worker_delta;
  double wake_ms=0,error=0;
};
int main(int argc,char** argv) {
  try {
    if(argc!=8)throw std::runtime_error("Usage: probe MODEL NATIVE_RECORDS OUTPUT MODE FRAMES PERIOD_US DIAGNOSTIC; MODE=single4|single5|shared|pinned|pinned_nospin");
    const std::string mode=argv[4];const int frames=std::stoi(argv[5]);
    const int period=std::stoi(argv[6]);const bool diagnostic=std::stoi(argv[7])!=0;
    const bool busy_pacing=std::getenv("PROBE_BUSY_PACING")!=nullptr;
    const bool fixed_input=std::getenv("PROBE_FIXED_INPUT")!=nullptr;
    const char* block_base=std::getenv("PROBE_BLOCK_BASE");
    const bool op_profile=std::getenv("PROBE_ORT_PROFILE")!=nullptr;
    if(frames<1||frames>10000||period<0||period>100000)throw std::runtime_error("invalid bounds");
    const bool single=mode=="single4"||mode=="single5";
    const bool split=mode=="pinned"||mode=="pinned_nospin";
    if(!single&&!split&&mode!="shared")throw std::runtime_error("invalid mode");
    if(single) {if(mode=="single4")pin(0,{4});else pin(0,{5});} else pin(0,{4,5});
    std::ifstream input(argv[2],std::ios::binary|std::ios::ate);
    if(!input)throw std::runtime_error("cannot open recording");
    const auto bytes=input.tellg();
    if(bytes<=0||static_cast<long long>(bytes)%sizeof(Record))throw std::runtime_error("invalid record size");
    std::vector<Record> records(static_cast<size_t>(bytes)/sizeof(Record));input.seekg(0);
    if(!input.read(reinterpret_cast<char*>(records.data()),bytes)||records.size()<2)throw std::runtime_error("truncated recording");
    for(const auto& r:records) {
      for(float x:r.obs)if(!std::isfinite(x))throw std::runtime_error("nonfinite observation");
      for(float x:r.raw)if(!std::isfinite(x))throw std::runtime_error("nonfinite reference");
    }
    const auto before=tids();
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"file_only_cpu_probe");
    Ort::SessionOptions opts;opts.SetIntraOpNumThreads(single?1:2);opts.SetInterOpNumThreads(1);
    opts.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    if(mode=="pinned_nospin")opts.AddConfigEntry("session.intra_op.allow_spinning","0");
    if(block_base)opts.AddConfigEntry("session.dynamic_block_base",block_base);
    if(op_profile)opts.EnableProfiling(argv[3]);
    Ort::Session session(env,argv[1],opts);
    const auto after=tids();std::vector<int> workers;
    for(int t:after)if(std::find(before.begin(),before.end(),t)==before.end())workers.push_back(t);
    if(workers.size()!=static_cast<size_t>(single?0:1))throw std::runtime_error("unexpected ORT worker count");
    if(split) {pin(workers[0],{5});pin(0,{4});}
    const int main_tid=getpid(),worker=single?0:workers[0];
    const std::string main_mask=affinity(main_tid),worker_mask=worker?affinity(worker):"";
    Ort::AllocatorWithDefaultOptions allocator;
    const auto in_name=session.GetInputNameAllocated(0,allocator),out_name=session.GetOutputNameAllocated(0,allocator);
    const char* ins[]={in_name.get()};const char* outs[]={out_name.get()};
    const std::array<int64_t,2> shape{1,993};
    auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
    const auto infer=[&](Record& record,std::array<float,31>& output) {
      auto tensor=Ort::Value::CreateTensor<float>(memory,record.obs.data(),993,shape.data(),shape.size());
      auto values=session.Run(Ort::RunOptions{nullptr},ins,&tensor,1,outs,1);
      if(values.size()!=1||values[0].GetTensorTypeAndShapeInfo().GetShape()!=std::vector<int64_t>({1,31}))throw std::runtime_error("invalid output shape");
      std::copy_n(values[0].GetTensorData<float>(),31,output.begin());
    };
    std::array<float,31> output{};for(int i=0;i<100;++i)infer(records[1+i%(records.size()-1)],output);
    std::vector<Row> rows(frames);std::vector<std::array<float,31>> actions(frames);
    bool parity=true;double max_error=0;
    const auto anchor=Clock::now()+std::chrono::milliseconds(100);
    for(int i=0;i<frames;++i) {
      auto& r=rows[i];r.frame=i;r.input=fixed_input?1:1+i%(records.size()-1);
      const auto deadline=anchor+std::chrono::microseconds(static_cast<long long>(i)*period);
      if(period) {
        if(busy_pacing) {while(Clock::now()<deadline){}}
        else std::this_thread::sleep_until(deadline);
      }
      r.wake_ms=period?std::chrono::duration<double,std::milli>(Clock::now()-deadline).count():0;
      Sched ma{},wa{};rusage usage_before{},usage_after{};long long pc=0,mc=0;
      if(diagnostic) {
        r.freq4_before=freq(4);r.freq5_before=freq(5);
        ma=scheduler(main_tid);if(worker)wa=scheduler(worker);
        getrusage(RUSAGE_SELF,&usage_before);pc=ns(CLOCK_PROCESS_CPUTIME_ID);mc=ns(CLOCK_THREAD_CPUTIME_ID);
      }
      r.cpu_before=sched_getcpu();r.start=ns();infer(records[r.input],actions[i]);r.end=ns();r.cpu_after=sched_getcpu();
      if(diagnostic) {
        r.main_cpu=ns(CLOCK_THREAD_CPUTIME_ID)-mc;r.process_cpu=ns(CLOCK_PROCESS_CPUTIME_ID)-pc;
        getrusage(RUSAGE_SELF,&usage_after);
        const auto mb=scheduler(main_tid);const auto wb=worker?scheduler(worker):Sched{};
        r.main_delta={mb.run-ma.run,mb.wait-ma.wait,mb.slices-ma.slices};
        r.worker_delta={wb.run-wa.run,wb.wait-wa.wait,wb.slices-wa.slices};
        r.voluntary=usage_after.ru_nvcsw-usage_before.ru_nvcsw;
        r.involuntary=usage_after.ru_nivcsw-usage_before.ru_nivcsw;
        r.freq4_after=freq(4);r.freq5_after=freq(5);
      }
      for(int j=0;j<31;++j) {
        const double ref=records[r.input].raw[j];const double error=std::abs(actions[i][j]-ref);
        if(!std::isfinite(actions[i][j])||error>1e-5+1e-4*std::abs(ref))parity=false;
        r.error=std::max(r.error,error);max_error=std::max(max_error,error);
      }
    }
    if(op_profile) {
      auto file=session.EndProfilingAllocated(allocator);
      std::cout<<"Operator profile: "<<file.get()<<'\n';
    }
    std::ofstream csv(std::string(argv[3])+".csv"),meta(std::string(argv[3])+".json");
    if(!csv||!meta)throw std::runtime_error("cannot create outputs");
    csv<<"frame,input,start_ns,end_ns,wall_ms,process_cpu_ms,main_cpu_ms,main_run_ms,main_wait_ms,worker_run_ms,worker_wait_ms,voluntary,involuntary,cpu_before,cpu_after,freq4_before,freq5_before,freq4_after,freq5_after,wake_ms,max_action_error\n"<<std::setprecision(12);
    std::vector<double> durations;
    for(const auto& r:rows) {
      durations.push_back((r.end-r.start)/1e6);
      csv<<r.frame<<','<<r.input<<','<<r.start<<','<<r.end<<','<<(r.end-r.start)/1e6<<','<<r.process_cpu/1e6<<','<<r.main_cpu/1e6<<','<<r.main_delta.run/1e6<<','<<r.main_delta.wait/1e6<<','<<r.worker_delta.run/1e6<<','<<r.worker_delta.wait/1e6<<','<<r.voluntary<<','<<r.involuntary<<','<<r.cpu_before<<','<<r.cpu_after<<','<<r.freq4_before<<','<<r.freq5_before<<','<<r.freq4_after<<','<<r.freq5_after<<','<<r.wake_ms<<','<<r.error<<'\n';
    }
    std::sort(durations.begin(),durations.end());
    meta<<std::setprecision(12)<<"{\"mode\":\""<<mode<<"\",\"frames\":"<<frames<<",\"period_us\":"<<period<<",\"diagnostic\":"<<(diagnostic?"true":"false")<<",\"ort_version\":\""<<Ort::GetVersionString()<<"\",\"main_tid\":"<<main_tid<<",\"worker_tid\":"<<worker<<",\"main_affinity\":\""<<main_mask<<"\",\"worker_affinity\":\""<<worker_mask<<"\",\"parity_passed\":"<<(parity?"true":"false")<<",\"max_action_error\":"<<max_error<<",\"p50_ms\":"<<durations[frames/2]<<",\"p99_ms\":"<<durations[std::min(frames-1,static_cast<int>(frames*.99))]<<",\"max_ms\":"<<durations.back()<<"}\n";
    csv.close();meta.close();if(!csv||!meta)throw std::runtime_error("output write failed");
    std::cout<<mode<<" diagnostic="<<diagnostic<<" p50="<<durations[frames/2]<<" max="<<durations.back()<<" parity="<<parity<<std::endl;
    return parity?0:2;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
