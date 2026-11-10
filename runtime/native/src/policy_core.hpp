#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sprite_policy {
constexpr int joints=31, frames=8, observation_size=795;
using Joints=std::array<double,joints>;
template<int HistoryFrames> struct PolicyCore {
  static_assert(HistoryFrames==8 || HistoryFrames==10,"unsupported policy history profile");
  static constexpr int history_frames=HistoryFrames;
  static constexpr int history_size=99*HistoryFrames;
  static constexpr int input_size=history_size+3;
  Joints defaults{},offset{},scale{},previous{},initial{};
  std::array<float,history_size> history{};
  unsigned steps=0,completed=0;
  bool ready=false;
  // Keep the affine profile, but never carry warmup/handshake history into control.
  void reset_episode() {
    previous={}; initial={}; history={}; completed=0; ready=false;
  }
  std::array<float,input_size> observe(const std::array<double,71>& input) {
    for(double v:input) if(!std::isfinite(v)) throw std::runtime_error("nonfinite input");
    const std::array<int,5> widths{31,31,31,3,3};
    int base=0;
    for(int term=0;term<5;++term) {
      const int width=widths[term];
      if(ready) for(int i=0;i<width*(HistoryFrames-1);++i) history[base+i]=history[base+width+i];
      for(int j=0;j<width;++j) {
        float value=0;
        if(term==0) value=static_cast<float>(input[j])-static_cast<float>(defaults[j]);
        if(term==1) value=static_cast<float>(input[31+j]);
        if(term==2) value=static_cast<float>(previous[j]);
        if(term==3) value=static_cast<float>(input[62+j]);
        if(term==4) value=static_cast<float>(input[65+j]);
        if(!std::isfinite(value)) throw std::runtime_error("observation overflow");
        for(int f=ready?HistoryFrames-1:0;f<HistoryFrames;++f) history[base+f*width+j]=value;
      }
      base+=width*HistoryFrames;
    }
    std::array<float,input_size> result{};
    std::copy(history.begin(),history.end(),result.begin());
    for(int j=0;j<3;++j) {
      result[history_size+j]=static_cast<float>(input[68+j]);
      if(!std::isfinite(result[history_size+j])) throw std::runtime_error("command overflow");
    }
    return result;
  }
  Joints latch(const std::array<float,31>& raw,const std::array<double,71>& input,
               bool policy_output_history=false) {
    if(!ready) for(int j=0;j<31;++j) {
      if(!std::isfinite(scale[j]) || std::abs(scale[j])<=1e-12)
        throw std::runtime_error("invalid action scale");
      initial[j]=(input[j]-offset[j])/scale[j];
    }
    double alpha=1;
    if(completed<steps) {
      ++completed;
      const double x=static_cast<double>(completed)/steps;
      alpha=x*x*(3-2*x);
    }
    Joints target{};
    for(int j=0;j<31;++j) {
      if(!std::isfinite(raw[j])) throw std::runtime_error("nonfinite actor output");
      const double blended=(1-alpha)*initial[j]+alpha*raw[j];
      target[j]=offset[j]+scale[j]*blended;
      // SuperMimic training stores actor output, independently of actuator
      // saturation or deployment handoff. Keep legacy behavior by default.
      previous[j]=policy_output_history?raw[j]:blended;
      if(!std::isfinite(target[j])) throw std::runtime_error("nonfinite target");
    }
    ready=true;
    return target;
  }
};
// Existing native/shadow/powered entries retain their original eight-frame ABI.
using Core=PolicyCore<8>;
using SuperMimicCore=PolicyCore<10>;
}
