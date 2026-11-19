#include "../src/policy_gamepad.hpp"
#include <cassert>
#include <iostream>

int main() {
  using namespace sprite_policy;
  static_assert(GamepadControlWindow::close_frame-GamepadControlWindow::open_frame==15000);
  static_assert(kXboxDurationSeconds==326.0);
  GamepadConfig cfg;cfg.xbox_control_040=true;cfg.forward=.40;cfg.acceleration=.10;
  GamepadCommand command(cfg);GamepadControlWindow window;
  GamepadSnapshot s{};s.connected=true;s.permitted=true;s.lt=1;
  std::size_t allowed_frames=0;bool cap=false;
  for(std::uint64_t frame=0;frame<16301;++frame) {
    auto now=(frame+1)*20000000ULL;s.stamp=now;
    s.forward=frame<1004?-1:0;
    if(frame==1004){s.permitted=false;s.lt=-1;}
    if(frame==1005){s.permitted=true;s.lt=1;}
    if(frame>1005){s.forward=-1;s.turn=-1;}
    bool allowed=window.allowed(frame,s,now);allowed_frames+=allowed;
    auto v=command.sample(s,now,allowed);
    if(frame<=1005)assert(v[0]==0 && v[2]==0);
    assert(v[0]>=0 && v[0]<=.40 && v[1]==0 && std::abs(v[2])<=.20);
    if(v[0]==.40)cap=true;
    if(frame==1205)assert(v[0]==.40);
    if(frame==16001)assert(v[0]==.40 && v[2]==.20);
    if(frame>=16203)assert(v[0]==0 && v[2]==0);
  }
  assert(cap && allowed_frames==14997);
  for(bool disconnect : {false,true}) {
    GamepadCommand c(cfg);GamepadControlWindow w;GamepadSnapshot p{};
    p.connected=true;p.lt=-1;
    for(std::uint64_t f=1002;f<1700;++f) {
      auto now=(f+1)*20000000ULL;p.stamp=now;
      if(f==1003){p.permitted=true;p.lt=1;}
      if(f>1003){p.forward=-1;p.turn=1;}
      if(f==1450){if(disconnect)p.connected=false;else {p.permitted=false;p.lt=-1;}}
      auto v=c.sample(p,now,w.allowed(f,p,now));
      if(f==1449)assert(v[0]==.40 && v[2]==-.20);
      if(f>=1651)assert(v[0]==0 && v[2]==0);
    }
  }
  GamepadConfig legacy;legacy.xbox_control_025=true;legacy.forward=.40;
  bool rejected=false;try{legacy.validate();}catch(...){rejected=true;}assert(rejected);
  auto excessive=cfg;excessive.forward=.41;rejected=false;
  try{excessive.validate();}catch(...){rejected=true;}assert(rejected);
  excessive=cfg;excessive.reverse=.06;rejected=false;
  try{excessive.validate();}catch(...){rejected=true;}assert(rejected);
  excessive=cfg;excessive.turn=.21;rejected=false;
  try{excessive.validate();}catch(...){rejected=true;}assert(rejected);
  GamepadControlWindow stale;s={};s.connected=true;s.lt=-1;s.stamp=1;
  assert(!stale.allowed(1002,s,200000000));
  std::cout<<"PASS 300-second window, 0.40 cap, fresh trigger, 4-second braking, disconnect, legacy caps\n";
}
