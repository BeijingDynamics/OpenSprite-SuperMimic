#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <limits>
#include <stdexcept>
#include <vector>
#include <time.h>
#include <linux/errqueue.h>
#include <linux/net_tstamp.h>
#include <sys/socket.h>
#include <time.h>

namespace sprite_policy {
struct ActionTxContext {
  std::uint64_t sequence=0;
  std::int64_t feedback_ns=0, accepted_ns=0;
  bool group=false, powered=false;
};
struct ActionTxEvent {
  ActionTxContext context;
  std::uint32_t id=0;
  int motor_id=0;
  std::int64_t call_begin_ns=0, call_end_ns=0;
  std::int64_t realtime_before_ns=0;
  std::int64_t software_realtime_ns=0, hardware_raw_ns=0;
  std::int64_t hardware_received_ns=0;
};
class ActionTxTrace {
public:
  bool enabled=false;
  ActionTxContext context;
  std::vector<ActionTxEvent> events;
  void prepare() { enabled=true; events.reserve(200000); }
  void sent(int motor_id, std::int64_t begin, std::int64_t end, std::int64_t realtime=0) {
    if (!enabled) return;
    if (events.size()==events.capacity()) throw std::runtime_error("TX evidence capacity exceeded");
    events.push_back({context,static_cast<std::uint32_t>(events.size()),motor_id,begin,end,realtime,0,0,0});
  }
  void stamp(std::uint32_t id,std::int64_t software,std::int64_t hardware,std::int64_t received=0) {
    if(id>=events.size()) throw std::runtime_error("TX timestamp unmatched packet ID");
    auto& event=events[id];
    if(software) {
      if(event.software_realtime_ns && event.software_realtime_ns!=software)
        throw std::runtime_error("TX software timestamp changed");
      event.software_realtime_ns=software;
    }
    if(hardware) {
      if(event.hardware_raw_ns && event.hardware_raw_ns!=hardware)
        throw std::runtime_error("TX hardware timestamp changed");
      event.hardware_raw_ns=hardware;
      if (!event.hardware_received_ns) event.hardware_received_ns=received;
    }
  }
  void drain(int fd) {
    if (!enabled) return;
    for (;;) {
      std::array<unsigned char,128> payload{};
      alignas(cmsghdr) std::array<unsigned char,512> control{};
      iovec buffer{payload.data(),payload.size()};
      msghdr message{};message.msg_iov=&buffer;message.msg_iovlen=1;
      message.msg_control=control.data();message.msg_controllen=control.size();
      const auto received=recvmsg(fd,&message,MSG_ERRQUEUE|MSG_DONTWAIT);
      if(received<0 && (errno==EAGAIN || errno==EWOULDBLOCK)) return;
      if(received<0 || message.msg_flags&(MSG_TRUNC|MSG_CTRUNC))
        throw std::runtime_error("TX timestamp queue error/truncation");
      timespec receipt{};
      if(clock_gettime(CLOCK_MONOTONIC,&receipt)) throw std::runtime_error("TX receipt clock failed");
      bool have_id=false,have_stamp=false;std::uint32_t id=0;
      std::array<timespec,3> stamps{};
      for(auto* c=CMSG_FIRSTHDR(&message);c;c=CMSG_NXTHDR(&message,c)) {
        if(c->cmsg_level==SOL_SOCKET && c->cmsg_type==SO_TIMESTAMPING &&
           c->cmsg_len>=CMSG_LEN(sizeof(stamps))) {
          std::memcpy(stamps.data(),CMSG_DATA(c),sizeof(stamps));have_stamp=true;
        } else if(c->cmsg_len>=CMSG_LEN(sizeof(sock_extended_err))) {
          sock_extended_err error{};std::memcpy(&error,CMSG_DATA(c),sizeof(error));
          if(error.ee_origin==SO_EE_ORIGIN_TIMESTAMPING) { id=error.ee_data;have_id=true; }
        }
      }
      if(!have_id || !have_stamp) throw std::runtime_error("TX timestamp missing identifier or stamp");
      const auto ns=[](const timespec& t) {return static_cast<std::int64_t>(t.tv_sec)*1000000000LL+t.tv_nsec;};
      stamp(id,ns(stamps[0]),ns(stamps[2]),ns(receipt));
    }
  }
};
inline std::int64_t realtime_now_ns() {
  timespec t{};
  if(clock_gettime(CLOCK_REALTIME,&t)) throw std::runtime_error("TX realtime clock read failed");
  return static_cast<std::int64_t>(t.tv_sec)*1000000000LL+t.tv_nsec;
}
inline bool coherent_commit_due(std::size_t slot,std::uint64_t pending,std::uint64_t active) {
  if(pending<active || pending>active+1) throw std::runtime_error("coherent action sequence skipped/regressed");
  return slot%4==0 && pending!=active;
}
}
