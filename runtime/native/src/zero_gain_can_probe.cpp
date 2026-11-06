// Finite CAN-FD transport test. Only zero-gain MIT frames; no enable/mode/policy path.
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <unistd.h>
#include <array>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <memory>
#include "readonly_state_server.hpp"

namespace {
using Clock=std::chrono::steady_clock;
std::uint64_t now_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();}
struct Motor {std::string name;int bus,id,master;unsigned position=32767;std::uint64_t tx=0,rx=0,last=0,max_gap=0,stamp=0;double pmin=0,pmax=0,vmin=0,vmax=0,q=0,v=0;};
struct Bus {std::string name;int fd=-1;std::vector<int> motors;std::size_t cursor=0;std::uint64_t due=0,period=0,skipped=0;};
struct Sample {std::uint64_t mono,software,hardware;int motor;std::array<unsigned char,8> data;};
canfd_frame zero(unsigned position){
    canfd_frame f{};f.len=8;f.flags=CANFD_BRS;
    f.data[0]=(position>>8)&255;f.data[1]=position&255;
    f.data[2]=0x7f;f.data[3]=0xf0;f.data[4]=0;f.data[5]=0;f.data[6]=0x07;f.data[7]=0xff;
    return f;
}
void validate(const canfd_frame& f){
    const auto* d=f.data;
    const unsigned v=(d[2]<<4)|(d[3]>>4),kp=((d[3]&15)<<8)|d[4],kd=(d[5]<<4)|(d[6]>>4),t=((d[6]&15)<<8)|d[7];
    if(f.len!=8 || f.flags!=CANFD_BRS || kp || kd || v!=2047 || t!=2047)
        throw std::runtime_error("nonzero-gain/velocity/torque frame rejected");
}
void disabled(const canfd_frame& f,int id){
    if(f.len!=8 || !(f.flags&CANFD_BRS) || (f.data[0]&15)!=id || (f.data[0]>>4)!=0)
        throw std::runtime_error("feedback is not FD+BRS/expected ID/disabled");
}
void self_test(){
    for(unsigned p:{0u,1u,32767u,65535u})validate(zero(p));
    for(int byte:{3,4,5,6,7}){auto f=zero(32767);f.data[byte]^=1;bool rejected=false;try{validate(f);}catch(...){rejected=true;}if(!rejected)throw std::runtime_error("unsafe field accepted");}
    canfd_frame f{};f.len=8;f.flags=CANFD_BRS;f.data[0]=1;disabled(f,1);
    f.data[0]=0x11;bool rejected=false;try{disabled(f,1);}catch(...){rejected=true;}if(!rejected)throw std::runtime_error("enabled status accepted");
    std::cout<<"ZERO_GAIN_CODEC_AND_DISABLED_GUARDS_PASS\n";
}
}

int main(int argc,char** argv){
    if(argc==2 && std::string(argv[1])=="--self-test"){try{self_test();return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
    if((argc!=6 && argc!=10) || std::string(argv[5])!="ZERO_GAIN_DISABLED_MOTORS_ONLY"){
        std::cerr<<"Usage: probe INVENTORY_TSV SECONDS RATE OUTPUT_PREFIX ZERO_GAIN_DISABLED_MOTORS_ONLY [--state-socket PATH --state-frames N]\n";return 2;
    }
    if(std::ifstream(std::string(argv[4])+".csv") || std::ifstream(std::string(argv[4])+".json")){std::cerr<<"output exists; refusing overwrite\n";return 2;}
    std::ofstream csv(std::string(argv[4])+".csv"),j(std::string(argv[4])+".json");
    if(!csv||!j){std::cerr<<"output path is not writable\n";return 2;}
    std::vector<Motor> motors;std::vector<Bus> buses;std::vector<Sample> samples;
    std::unique_ptr<sprite_policy::ReadonlyStateServer> state_server;std::uint64_t state_frames=0,states_sent=0,reverse_bytes=0;bool peer_half_closed=false;
    std::string error;double seconds=0,rate=0;std::uint64_t discarded=0,start=0,finish=0;
    try{
        seconds=std::stod(argv[2]);rate=std::stod(argv[3]);
        if(!std::isfinite(seconds)||seconds<.1||seconds>120||!std::isfinite(rate)||rate<=0||rate>500)throw std::runtime_error("duration/rate outside bounded probe");
        if(argc==10){
            if(std::string(argv[6])!="--state-socket" || std::string(argv[8])!="--state-frames")throw std::runtime_error("invalid read-only options");
            std::size_t used=0;state_frames=std::stoull(argv[9],&used);
            if(used!=std::strlen(argv[9])||!state_frames||state_frames>static_cast<std::uint64_t>(seconds*50)-1)throw std::runtime_error("state frame count exceeds bounded run");
        }
        std::ifstream cfg(argv[1]);if(!cfg)throw std::runtime_error("inventory missing");
        std::string iface,name,line;int id,master;
        while(std::getline(cfg,line)){
            if(line.empty())continue;
            std::istringstream fields(line);if(!(fields>>iface>>name>>id>>master))throw std::runtime_error("invalid inventory row");
            if(iface!="kcan1"&&iface!="kcan2"&&iface!="kcan3"&&iface!="kcan4")throw std::runtime_error("unknown bus");
            if(id<1||id>8||master!=id+16||name.find_first_not_of("abcdefghijklmnopqrstuvwxyz_0123456789")!=std::string::npos)throw std::runtime_error("invalid endpoint");
            auto it=std::find_if(buses.begin(),buses.end(),[&](const Bus& b){return b.name==iface;});
            int bi=static_cast<int>(it-buses.begin());if(it==buses.end()){Bus b;b.name=iface;buses.push_back(b);}
            for(const auto& m:motors)if(m.name==name || (m.bus==bi && m.id==id))throw std::runtime_error("duplicate motor");
            Motor m;m.name=name;m.bus=bi;m.id=id;m.master=master;
            if(state_frames){
                if(!(fields>>m.pmin>>m.pmax>>m.vmin>>m.vmax)||!std::isfinite(m.pmin)||!std::isfinite(m.pmax)||!std::isfinite(m.vmin)||!std::isfinite(m.vmax)||m.pmin>=m.pmax||m.vmin>=m.vmax)throw std::runtime_error("read-only state requires exact protocol position/velocity ranges");
            }
            std::string extra;if(fields>>extra)throw std::runtime_error("unexpected inventory columns");
            buses[bi].motors.push_back(static_cast<int>(motors.size()));motors.push_back(m);
        }
        if(motors.size()!=31||buses.size()!=4)throw std::runtime_error("full31/four-bus inventory required");
        if(state_frames){
            std::array<std::string,31> names{};for(std::size_t i=0;i<31;++i)names[i]=motors[i].name;
            state_server=std::make_unique<sprite_policy::ReadonlyStateServer>(argv[7],sprite_policy::name_hash(names));
            state_server->accept_peer(); // No CAN transmission until the actor's SHUT_WR is verified.
        }
        samples.reserve(static_cast<std::size_t>(seconds*rate*31*1.1));
        for(auto& b:buses){
            if(b.motors.empty()||b.motors.size()>8)throw std::runtime_error("invalid bus population");
            b.fd=::socket(PF_CAN,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,CAN_RAW);if(b.fd<0)throw std::runtime_error("CAN socket failed");
            const int one=1,flags=SOF_TIMESTAMPING_RX_HARDWARE|SOF_TIMESTAMPING_RX_SOFTWARE|SOF_TIMESTAMPING_SOFTWARE|SOF_TIMESTAMPING_RAW_HARDWARE;
            if(setsockopt(b.fd,SOL_CAN_RAW,CAN_RAW_FD_FRAMES,&one,sizeof(one))||setsockopt(b.fd,SOL_SOCKET,SO_TIMESTAMPING,&flags,sizeof(flags)))throw std::runtime_error("FD/timestamp setup failed");
            ifreq req{};std::strncpy(req.ifr_name,b.name.c_str(),IFNAMSIZ-1);if(ioctl(b.fd,SIOCGIFINDEX,&req))throw std::runtime_error("CAN interface missing");
            sockaddr_can address{};address.can_family=AF_CAN;address.can_ifindex=req.ifr_ifindex;
            if(::bind(b.fd,reinterpret_cast<sockaddr*>(&address),sizeof(address)))throw std::runtime_error("CAN bind failed");
            b.period=static_cast<std::uint64_t>(1e9/(rate*b.motors.size()));
        }
        start=now_ns();const auto end=start+static_cast<std::uint64_t>(seconds*1e9);
        auto state_due=start+20000000;
        for(auto& m:motors)m.last=start;
        for(auto& b:buses)b.due=start;
        std::vector<pollfd> polls;for(const auto& b:buses)polls.push_back({b.fd,POLLIN,0});
        while(now_ns()<end && (!state_server || states_sent<state_frames)){
            auto now=now_ns();
            for(auto& b:buses){
                if(now>=b.due){
                    auto& m=motors[b.motors[b.cursor]];auto f=zero(m.position);f.can_id=m.id;validate(f);
                    if(::write(b.fd,&f,CANFD_MTU)!=CANFD_MTU)throw std::runtime_error("CAN zero-gain write failed");
                    ++m.tx;b.cursor=(b.cursor+1)%b.motors.size();b.due+=b.period;
                    if(now>=b.due){const auto missed=(now-b.due)/b.period+1;b.skipped+=missed;b.due+=missed*b.period;}
                }
            }
            now=now_ns();auto due=end;for(const auto& b:buses)due=std::min(due,b.due);
            const auto wait=due>now?std::min<std::uint64_t>(due-now,1000000):0;
            timespec timeout{0,static_cast<long>(wait)};
            const int result=ppoll(polls.data(),polls.size(),&timeout,nullptr);
            if(result<0 && errno!=EINTR)throw std::runtime_error("CAN poll failed");
            for(std::size_t bi=0;bi<buses.size();++bi){
                if(polls[bi].revents&(POLLERR|POLLHUP|POLLNVAL))throw std::runtime_error("CAN socket error");
                for(int drain=0;drain<32 && (polls[bi].revents&POLLIN);++drain){
                    canfd_frame f{};alignas(cmsghdr) std::array<char,CMSG_SPACE(3*sizeof(timespec))> control{};
                    iovec io{&f,sizeof(f)};msghdr msg{};msg.msg_iov=&io;msg.msg_iovlen=1;msg.msg_control=control.data();msg.msg_controllen=control.size();
                    const auto n=recvmsg(buses[bi].fd,&msg,MSG_DONTWAIT);
                    if(n<0 && (errno==EAGAIN||errno==EWOULDBLOCK))break;
                    if(n!=CANFD_MTU || (f.can_id&(CAN_EFF_FLAG|CAN_RTR_FLAG|CAN_ERR_FLAG)))throw std::runtime_error("non-FD/standard frame");
                    auto it=std::find_if(motors.begin(),motors.end(),[&](const Motor& m){return m.bus==static_cast<int>(bi)&&m.master==static_cast<int>(f.can_id);});
                    if(it==motors.end())throw std::runtime_error("unexpected CAN endpoint");
                    disabled(f,it->id);std::uint64_t software=0,hardware=0;
                    for(auto* c=CMSG_FIRSTHDR(&msg);c;c=CMSG_NXTHDR(&msg,c))if(c->cmsg_level==SOL_SOCKET&&c->cmsg_type==SO_TIMESTAMPING&&c->cmsg_len>=CMSG_LEN(3*sizeof(timespec))){
                        std::array<timespec,3> ts{};std::memcpy(ts.data(),CMSG_DATA(c),sizeof(ts));
                        software=static_cast<std::uint64_t>(ts[0].tv_sec)*1000000000+ts[0].tv_nsec;hardware=static_cast<std::uint64_t>(ts[2].tv_sec)*1000000000+ts[2].tv_nsec;
                    }
                    if(!software||!hardware){++discarded;continue;}
                    const auto received=now_ns();it->max_gap=std::max(it->max_gap,received-it->last);it->last=received;++it->rx;it->position=(f.data[1]<<8)|f.data[2];
                    if(state_server){
                        const auto realtime=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                        if(software>static_cast<std::uint64_t>(realtime)||static_cast<std::uint64_t>(realtime)-software>30000000)throw std::runtime_error("kernel motor feedback timestamp stale/future");
                        it->stamp=received-(static_cast<std::uint64_t>(realtime)-software);
                        it->q=it->position*(it->pmax-it->pmin)/65535.+it->pmin;
                        const unsigned raw_v=(f.data[3]<<4)|(f.data[4]>>4);it->v=raw_v*(it->vmax-it->vmin)/4095.+it->vmin;
                    }
                    Sample s{received,software,hardware,static_cast<int>(it-motors.begin()),{}};std::copy_n(f.data,8,s.data.begin());samples.push_back(s);
                }
            }
            now=now_ns();for(const auto& m:motors)if(now-m.last>200000000)throw std::runtime_error("motor feedback stale: "+m.name);
            if(state_server && now>=state_due){
                sprite_policy::ProtectionVector q{},v{};auto oldest=now;
                for(std::size_t i=0;i<31;++i){if(!motors[i].stamp)throw std::runtime_error("motor state missing before publish");oldest=std::min(oldest,motors[i].stamp);q[i]=motors[i].q;v[i]=motors[i].v;}
                state_server->publish(q,v,oldest,now);++states_sent;state_due+=20000000;
                if(now>=state_due)throw std::runtime_error("50Hz state producer missed a complete period");
            }
        }
    }catch(const std::exception& e){error=e.what();}
    finish=now_ns();for(auto& b:buses)if(b.fd>=0){::close(b.fd);b.fd=-1;}
    if(state_server){reverse_bytes=state_server->reverse_bytes();peer_half_closed=state_server->write_half_closed();state_server.reset();}
    bool passed=error.empty()&&motors.size()==31&&(!state_frames||states_sent==state_frames);std::uint64_t tx=0,rx=0;
    const double elapsed=start?(finish-start)/1e9:0;
    for(const auto& m:motors){tx+=m.tx;rx+=m.rx;if(m.rx<(state_frames?elapsed:seconds)*rate*.95)passed=false;}
    csv<<"monotonic_ns,software_timestamp_ns,hardware_timestamp_ns,motor_index,payload_hex\n";
    const char* hex="0123456789abcdef";
    for(const auto& s:samples){csv<<s.mono<<','<<s.software<<','<<s.hardware<<','<<s.motor<<',';for(auto c:s.data)csv<<hex[c>>4]<<hex[c&15];csv<<'\n';}
    j<<"{\"passed\":"<<(passed?"true":"false")<<",\"mode\":\"native_zero_gain_transport_only\",\"seconds\":"<<seconds<<",\"elapsed_s\":"<<elapsed<<",\"requested_rate_hz\":"<<rate<<",\"tx\":"<<tx<<",\"rx\":"<<rx<<",\"discarded_timestamp_frames\":"<<discarded<<",\"enable_attempts\":0,\"nonzero_gain_or_feedforward_tx\":0,\"physical_imu_access\":false,\"policy_inference\":false,\"state_frames_sent\":"<<states_sent<<",\"state_socket_reverse_bytes\":"<<reverse_bytes<<",\"actor_write_half_closed\":"<<(peer_half_closed?"true":"false")<<",\"error\":\""<<error<<"\",\"motors\":[";
    for(std::size_t i=0;i<motors.size();++i){const auto& m=motors[i];if(i)j<<',';j<<"{\"name\":\""<<m.name<<"\",\"tx\":"<<m.tx<<",\"rx\":"<<m.rx<<",\"actual_rx_hz\":"<<(elapsed?m.rx/elapsed:0)<<",\"max_feedback_gap_ms\":"<<m.max_gap/1e6<<'}';}
    j<<"],\"buses\":[";for(std::size_t i=0;i<buses.size();++i){if(i)j<<',';j<<"{\"interface\":\""<<buses[i].name<<"\",\"skipped_slots\":"<<buses[i].skipped<<'}';}j<<"]}\n";
    csv.flush();j.flush();if(!csv||!j)return 1;
    std::cout<<"ZERO_GAIN_NATIVE_RESULT passed="<<passed<<" tx="<<tx<<" rx="<<rx<<" elapsed="<<elapsed<<" error="<<error<<'\n';
    return passed?0:1;
}
