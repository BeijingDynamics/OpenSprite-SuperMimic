#include "../src/action_tx_trace.hpp"
#include <cassert>
#include <iostream>
int main() {
  using namespace sprite_policy;
  // An arriving target must wait for at most the next 2 ms mapping boundary,
  // not the next 20 ms low-rate cycle. No skipped sequence is admissible.
  for (std::size_t arrival=0; arrival<400; ++arrival) {
    auto slot=arrival;
    while (!coherent_commit_due(slot,2,1)) ++slot;
    assert(slot-arrival<=3);
    assert(!coherent_commit_due(slot,2,2));
  }
  for (auto pending : {0u,3u}) {
    bool rejected=false;
    try { (void)coherent_commit_due(4,pending,1); }
    catch (const std::runtime_error&) { rejected=true; }
    assert(rejected);
  }
  ActionTxTrace trace;trace.prepare();
  trace.context={2,10,20,true,false};
  trace.sent(7,30,31);trace.sent(8,32,33);
  // Hardware and software completions may arrive separately and out of order.
  trace.stamp(1,0,500);trace.stamp(0,400,0);trace.stamp(1,401,0);trace.stamp(0,0,499);
  assert(trace.events[0].hardware_raw_ns==499 && trace.events[1].software_realtime_ns==401);
  assert(trace.events[1].context.sequence==2 && !trace.events[1].context.powered);
  bool rejected=false;
  try {trace.stamp(2,1,0);} catch(const std::runtime_error&) {rejected=true;}
  assert(rejected);
  std::cout << "coherent sequence and TX identity tests passed\n";
}
