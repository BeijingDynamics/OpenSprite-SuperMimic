#pragma once
// Optional per-inference timing. Records are buffered and flushed after control exits.
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sched.h>
#include <sys/resource.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <vector>

namespace sprite_diagnostics {
inline std::vector<int> tids() {
  std::vector<int> out;
  for(const auto& e:std::filesystem::directory_iterator("/proc/self/task"))
    out.push_back(std::stoi(e.path().filename()));
  std::sort(out.begin(),out.end());return out;
}
inline long long clock_ns(clockid_t id) {
  timespec t{};if(clock_gettime(id,&t))throw std::runtime_error("diagnostic clock failed");
  return t.tv_sec*1000000000LL+t.tv_nsec;
}
struct Scheduled {long long run=0,wait=0,slices=0;};
inline Scheduled scheduled(int fd) {
  char b[160]{};if(pread(fd,b,sizeof(b)-1,0)<=0)throw std::runtime_error("diagnostic schedstat read failed");
  Scheduled s;if(std::sscanf(b,"%lld %lld %lld",&s.run,&s.wait,&s.slices)!=3)
    throw std::runtime_error("diagnostic schedstat parse failed");
  return s;
}
struct Snapshot {long long main_cpu=0,process_cpu=0,voluntary=0,involuntary=0;Scheduled main,worker;int cpu=0;};
struct Row {unsigned frame=0;long long start=0,end=0;Snapshot a,b;};
class Probe {
  std::string path_;int main_fd_=-1,worker_fd_=-1;std::vector<Row> rows_;std::size_t size_=0;
  Snapshot sample() {
    Snapshot s;s.main=scheduled(main_fd_);s.worker=scheduled(worker_fd_);
    rusage usage{};if(getrusage(RUSAGE_SELF,&usage))throw std::runtime_error("diagnostic rusage failed");
    s.voluntary=usage.ru_nvcsw;s.involuntary=usage.ru_nivcsw;s.cpu=sched_getcpu();
    s.main_cpu=clock_ns(CLOCK_THREAD_CPUTIME_ID);s.process_cpu=clock_ns(CLOCK_PROCESS_CPUTIME_ID);
    return s;
  }
public:
  Probe(std::string path,std::size_t count,const std::vector<int>& before):path_(std::move(path)),rows_(count) {
    std::vector<int> workers;for(int tid:tids())
      if(std::find(before.begin(),before.end(),tid)==before.end())workers.push_back(tid);
    if(workers.size()!=1)throw std::runtime_error("diagnostic expected exactly one ORT worker");
    main_fd_=open(("/proc/self/task/"+std::to_string(getpid())+"/schedstat").c_str(),O_RDONLY|O_CLOEXEC);
    worker_fd_=open(("/proc/self/task/"+std::to_string(workers[0])+"/schedstat").c_str(),O_RDONLY|O_CLOEXEC);
    if(main_fd_<0||worker_fd_<0)throw std::runtime_error("diagnostic schedstat open failed");
    std::ofstream meta(path_+".threads.json");
    meta<<"{\"main_tid\":"<<getpid()<<",\"ort_worker_tid\":"<<workers[0]<<"}\n";
  }
  void begin(unsigned frame) {
    if(size_>=rows_.size())throw std::runtime_error("diagnostic buffer exhausted");
    auto& r=rows_[size_];r.frame=frame;r.a=sample();r.start=clock_ns(CLOCK_MONOTONIC);
  }
  void end() {auto& r=rows_[size_];r.end=clock_ns(CLOCK_MONOTONIC);r.b=sample();++size_;}
  ~Probe() noexcept {
    if(main_fd_>=0)close(main_fd_);
    if(worker_fd_>=0)close(worker_fd_);
    try {
      std::ofstream out(path_+".inferdiag.csv");
      out<<"frame,start_ns,end_ns,main_cpu_ns,process_cpu_ns,main_run_ns,main_wait_ns,worker_run_ns,worker_wait_ns,voluntary,involuntary,cpu_before,cpu_after\n";
      for(std::size_t i=0;i<size_;++i){const auto& r=rows_[i];const auto& a=r.a;const auto& b=r.b;
        out<<r.frame<<','<<r.start<<','<<r.end<<','<<b.main_cpu-a.main_cpu<<','<<b.process_cpu-a.process_cpu
           <<','<<b.main.run-a.main.run<<','<<b.main.wait-a.main.wait<<','<<b.worker.run-a.worker.run
           <<','<<b.worker.wait-a.worker.wait<<','<<b.voluntary-a.voluntary<<','<<b.involuntary-a.involuntary
           <<','<<a.cpu<<','<<b.cpu<<'\n';}
      if(!out)std::fprintf(stderr,"Inference diagnostic output failed\n");
    }catch(...){std::fprintf(stderr,"Inference diagnostic flush failed\n");}
  }
};
}
