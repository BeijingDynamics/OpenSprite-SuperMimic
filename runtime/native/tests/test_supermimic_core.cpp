#include "../src/policy_core.hpp"
#include "../src/policy_protection.hpp"
#include "../src/sling_window.hpp"
#include <cassert>
#include <limits>

int main() {
  static_assert(sprite_policy::Core::input_size==795,"legacy ABI changed");
  static_assert(sprite_policy::SuperMimicCore::input_size==993,"SuperMimic ABI changed");
  sprite_policy::SuperMimicCore core;core.scale.fill(.25);core.defaults.fill(.125);
  core.offset.fill(.5);core.steps=2;
  std::array<double,71> row{};row[0]=1;row[31]=2;row[62]=3;row[67]=-1;row[68]=-.3;
  auto obs=core.observe(row);
  for(int f=0;f<10;++f) {
    assert(obs[31*f]==.875f);assert(obs[310+31*f]==2);
    assert(obs[620+31*f]==0);assert(obs[930+3*f]==3);assert(obs[960+3*f+2]==-1);
  }
  assert(obs[990]==static_cast<float>(-.3));
  std::array<float,31> raw{};raw[0]=4;
  auto target=core.latch(raw,row);assert(core.previous[0]==3);assert(target[0]==1.25);
  row[0]=2;obs=core.observe(row);
  assert(obs[0]==.875f && obs[279]==1.875f && obs[899]==3);
  target=core.latch(raw,row);assert(target[0]==1.5 && core.completed==2);
  // Roll through more than one full window, with distinct states and commands.
  for(int i=0;i<12;++i) {
    row[0]=10+i;row[68]=-i;obs=core.observe(row);core.latch(raw,row);
  }
  assert(obs[0]==11.875f && obs[279]==20.875f && obs[990]==-11);
  row[0]=std::numeric_limits<double>::infinity();bool rejected=false;
  try{core.observe(row);}catch(const std::runtime_error&){rejected=true;}assert(rejected);
  row[0]=0;raw[0]=std::numeric_limits<float>::quiet_NaN();rejected=false;
  try{core.latch(raw,row);}catch(const std::runtime_error&){rejected=true;}assert(rejected);

  using Joint=sprite_policy::StartupJoint;
  std::array<Joint,31> roles{};
  std::fill_n(roles.begin(),10,Joint::stance);
  roles[10]=roles[11]=Joint::left_ankle;
  roles[12]=roles[13]=Joint::right_ankle;
  std::fill(roles.end()-9,roles.end(),Joint::hold);
  sprite_policy::TrackedStartup startup(roles);
  sprite_policy::ProtectionVector measured{}, velocity{};
  measured[0]=.32;
  sprite_policy::ImpedanceTarget goal;
  goal.position=measured;
  goal.position[0]=.75;
  goal.position[30]=.5;
  goal.kp[0]=goal.kp[1]=26;
  goal.kp[30]=.02;
  auto output=startup.apply(goal,measured,velocity);
  assert(output.position==measured && output.kp[0]==0);
  for(int i=1;i<120;++i) output=startup.apply(goal,measured,velocity);
  measured[0]-=.20;
  for(int i=120;i<250;++i) {
    output=startup.apply(goal,measured,velocity);
    assert(output.feedforward==sprite_policy::ProtectionVector{});
  }
  assert(startup.complete() && startup.alpha()==1 && output.kp[0]==26);
  assert(output.position[0]==goal.position[0] && !startup.settled());
  goal.position[0]=.95;
  goal.position[12]=-.5;
  output=startup.apply(goal,measured,velocity);
  assert(output.position[0]==goal.position[0] && output.position[12]==goal.position[12]);
  assert(output.position[30]==0. && output.kp[30]==.02);
  measured=output.position;
  for(int i=0;i<50;++i) output=startup.apply(goal,measured,velocity);
  assert(startup.settled());
  velocity[0]=.4;
  output=startup.apply(goal,measured,velocity);
  assert(!startup.settled());

  sprite_policy::TrackedStartup with_notice(roles, sprite_policy::kSlingStartupHoldSeconds);
  velocity={};
  for(int i=0;i<120;++i) {
    output=with_notice.apply(goal,measured,velocity);
    assert(output.position==measured && output.kp==sprite_policy::ProtectionVector{} &&
           output.kd==sprite_policy::ProtectionVector{});
  }
  output=with_notice.apply(goal,measured,velocity);
  assert(output.kp[0]>0 && !with_notice.complete());
  for(int i=1;i<200;++i) output=with_notice.apply(goal,measured,velocity);
  assert(with_notice.complete() && output.kp==goal.kp);

  sprite_policy::TrackedStartup all_joints(roles, 0., true);
  measured={};velocity={};goal.position.fill(.1);goal.kp.fill(130.);goal.kd.fill(5.);
  for(int i=0;i<200;++i) output=all_joints.apply(goal,measured,velocity);
  assert(output.position==goal.position && output.kp==goal.kp && output.kd==goal.kd);
  assert(!all_joints.settled());
  measured=goal.position;
  for(int i=0;i<50;++i) output=all_joints.apply(goal,measured,velocity);
  assert(all_joints.settled());
  measured[30]-=.2;
  output=all_joints.apply(goal,measured,velocity);
  assert(!all_joints.settled() && output.position[30]==goal.position[30]);

  sprite_policy::SuperMimicCore trained_history;
  trained_history.scale.fill(.25);trained_history.offset.fill(.5);trained_history.steps=2;
  row={};row[0]=1.;row[67]=-1.;raw={};raw[0]=4.;
  trained_history.observe(row);
  target=trained_history.latch(raw,row,true);
  assert(target[0]==1.25 && trained_history.previous[0]==4.);
  // Actual measured state is distinct from either the request or its projection.
  row[0]=.7;obs=trained_history.observe(row);
  assert(obs[279]==static_cast<float>(.7) && obs[899]==4.f);
  raw[0]=-2.;trained_history.latch(raw,row,true);
  row[0]=.8;obs=trained_history.observe(row);
  assert(obs[868]==4.f && obs[899]==-2.f && obs[279]==static_cast<float>(.8));

  // A populated pre-control episode must never leak into direct handover.
  trained_history.reset_episode(); trained_history.steps=0;
  row[0]=.35; row[31]=.2;
  obs=trained_history.observe(row);
  for(int f=0;f<10;++f) {
    assert(obs[f*31]==static_cast<float>(.35));
    assert(obs[310+f*31]==static_cast<float>(.2));
    for(int j=0;j<31;++j) assert(obs[620+f*31+j]==0.f);
  }
  raw[0]=4.; target=trained_history.latch(raw,row,true);
  assert(target[0]==1.5 && trained_history.previous[0]==4.);
  row[0]=.36;obs=trained_history.observe(row);
  assert(obs[279]==static_cast<float>(.36) && obs[899]==4.f);
  sprite_policy::TrackedStartup direct(roles, 2.4, true, true);
  assert(!direct.captured());
  goal.position.fill(.8); goal.kp.fill(130.); goal.kd.fill(5.);
  output=direct.apply(goal,measured,velocity);
  assert(direct.complete() && direct.alpha()==1.);
  assert(output.position==goal.position && output.kp==goal.kp && output.kd==goal.kd);
  assert(output.feedforward==sprite_policy::ProtectionVector{});
  assert(!direct.settled()); // Large tracking error cannot silently pause or soften control.
  rejected=false;
  try { sprite_policy::TrackedStartup invalid(roles, 2.4, false, true); }
  catch(const std::runtime_error&) { rejected=true; }
  assert(rejected);

}
