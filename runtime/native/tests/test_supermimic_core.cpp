#include "../src/policy_core.hpp"
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
}
