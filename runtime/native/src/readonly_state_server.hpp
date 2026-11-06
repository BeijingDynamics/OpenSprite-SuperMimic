#pragma once
#include "policy_state_ipc.hpp"
#include <sys/stat.h>
#include <array>
#include <string>

namespace sprite_policy {
// This server accepts only a peer whose write half is closed. There is no target decoder.
class ReadonlyStateServer {
    int server_=-1,client_=-1;
    std::string path_;
    bool bound_=false;
    std::uint64_t hash_,sequence_=0,reverse_bytes_=0;
    bool write_half_closed_=false;
    void cleanup() noexcept {
        if(client_>=0)::close(client_);
        if(server_>=0)::close(server_);
        if(bound_)::unlink(path_.c_str());
        client_=server_=-1;bound_=false;
    }
public:
    ReadonlyStateServer(const std::string& path,std::uint64_t hash):path_(path),hash_(hash){
        try {
            sockaddr_un address{};address.sun_family=AF_UNIX;
            if(path.empty()||path.size()>=sizeof(address.sun_path))throw std::runtime_error("invalid read-only socket path");
            std::memcpy(address.sun_path,path.c_str(),path.size()+1);
            server_=::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC|SOCK_NONBLOCK,0);
            if(server_<0||::bind(server_,reinterpret_cast<sockaddr*>(&address),sizeof(address)))throw std::runtime_error("read-only bind failed; existing socket is never removed");
            bound_=true;
            if(::chmod(path.c_str(),0600)||::listen(server_,1))throw std::runtime_error("read-only socket permissions/listen failed");
        }catch(...){cleanup();throw;}
    }
    ~ReadonlyStateServer(){cleanup();}
    ReadonlyStateServer(const ReadonlyStateServer&)=delete;
    void accept_peer(){
        pollfd event{server_,POLLIN,0};
        if(::poll(&event,1,10000)!=1)throw std::runtime_error("read-only actor connection timeout");
        client_=::accept4(server_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
        if(client_<0)throw std::runtime_error("read-only accept failed");
        ucred peer{};socklen_t length=sizeof(peer);
        if(::getsockopt(client_,SOL_SOCKET,SO_PEERCRED,&peer,&length)||peer.uid!=::geteuid())throw std::runtime_error("read-only peer UID mismatch");
        pollfd incoming{client_,POLLIN,0};
        if(::poll(&incoming,1,1000)!=1)throw std::runtime_error("actor must close its write half before CAN starts");
        char byte;
        const auto n=::recv(client_,&byte,1,MSG_DONTWAIT);if(n>0)reverse_bytes_+=n;
        if(n!=0)throw std::runtime_error("actor attempted reverse traffic or omitted SHUT_WR");
        write_half_closed_=true;
    }
    void publish(const ProtectionVector& q,const ProtectionVector& v,std::uint64_t stamp,std::uint64_t now){
        PolicyState state;state.sequence=sequence_+1;state.stamp=stamp;state.position=q;state.velocity=v;
        state_fresh(state,now,30000000);
        char reverse;
        const auto n=::recv(client_,&reverse,1,MSG_DONTWAIT);if(n>0)reverse_bytes_+=n;
        if(n!=0)throw std::runtime_error("read-only actor write direction invariant failed");
        std::array<unsigned char,528> p{};std::memcpy(p.data(),"SPRT\1\0\1\0",8);
        ipc_put64(p.data()+8,state.sequence);ipc_put64(p.data()+16,stamp);ipc_put64(p.data()+24,hash_);
        for(std::size_t i=0;i<62;++i){double x=i<31?q[i]:v[i-31];if(!std::isfinite(x))throw std::runtime_error("nonfinite native motor state");std::uint64_t bits;std::memcpy(&bits,&x,8);ipc_put64(p.data()+32+8*i,bits);}
        if(::send(client_,p.data(),p.size(),MSG_DONTWAIT|MSG_NOSIGNAL)!=static_cast<ssize_t>(p.size()))throw std::runtime_error("state consumer failed/backpressure");
        ++sequence_;
    }
    std::uint64_t frames()const{return sequence_;}
    std::uint64_t reverse_bytes()const{return reverse_bytes_;}
    bool write_half_closed()const{return write_half_closed_;}
};
}
