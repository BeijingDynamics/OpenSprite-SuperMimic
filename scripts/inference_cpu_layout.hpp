#pragma once
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace sprite_schedule {
inline void pin_thread(pthread_t thread,int cpu) {
  cpu_set_t set,actual;CPU_ZERO(&set);CPU_SET(cpu,&set);CPU_ZERO(&actual);
  if(pthread_setaffinity_np(thread,sizeof(set),&set) ||
     pthread_getaffinity_np(thread,sizeof(actual),&actual) || !CPU_EQUAL(&set,&actual))
    throw std::runtime_error("inference IO thread affinity verification failed");
}
inline void pin_tid(int tid,int cpu) {
  cpu_set_t set,actual;CPU_ZERO(&set);CPU_SET(cpu,&set);CPU_ZERO(&actual);
  if(sched_setaffinity(tid,sizeof(set),&set) || sched_getaffinity(tid,sizeof(actual),&actual) ||
     !CPU_EQUAL(&set,&actual))throw std::runtime_error("inference compute affinity verification failed");
}
inline std::vector<int> tids() {
  std::vector<int> out;for(const auto& e:std::filesystem::directory_iterator("/proc/self/task"))
    out.push_back(std::stoi(e.path().filename()));
  std::sort(out.begin(),out.end());return out;
}
inline int isolate_compute(const std::vector<int>& before,int threads) {
  if(threads!=2)throw std::runtime_error("isolated inference requires exactly two ORT threads");
  std::vector<int> workers;for(int tid:tids())
    if(std::find(before.begin(),before.end(),tid)==before.end())workers.push_back(tid);
  if(workers.size()!=1)throw std::runtime_error("unexpected ORT worker count for isolated inference");
  pin_tid(workers[0],5);pin_tid(getpid(),4);return workers[0];
}
inline void verify_and_save(const std::string& path,int ort,int expected_io) {
  std::ofstream out(path+".affinity.json");int io=0;
  out<<"{\"main_cpu\":4,\"ort_worker_cpu\":5,\"io_cpu\":1,\"threads\":[";
  bool first=true;
  for(int tid:tids()) {
    const int wanted=tid==getpid()?4:tid==ort?5:1;
    if(wanted==1)++io;
    cpu_set_t expected,actual;CPU_ZERO(&expected);CPU_SET(wanted,&expected);CPU_ZERO(&actual);
    if(sched_getaffinity(tid,sizeof(actual),&actual)||!CPU_EQUAL(&expected,&actual))
      throw std::runtime_error("final inference thread layout mismatch");
    if(!first)out<<',';
    first=false;out<<"{\"tid\":"<<tid<<",\"cpu\":"<<wanted<<"}";
  }
  if(io!=expected_io)throw std::runtime_error("unexpected inference IO thread count");
  out<<"],\"verified\":true}\n";out.close();
  if(!out)throw std::runtime_error("inference affinity evidence write failed");
}
}
