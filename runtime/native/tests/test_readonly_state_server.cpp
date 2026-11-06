#include "../src/readonly_state_server.hpp"
#include <thread>
#include <chrono>
#include <cassert>
#include <memory>
#include <cstdlib>

int main(){
    char pattern[]="/tmp/sprite_readonly_test_XXXXXX";const char* d=mkdtemp(pattern);assert(d);
    const std::string path=std::string(d)+"/state.sock";
    const auto client=[&](int fault){
        int fd=socket(AF_UNIX,SOCK_SEQPACKET,0);assert(fd>=0);sockaddr_un a{};a.sun_family=AF_UNIX;std::strcpy(a.sun_path,path.c_str());assert(connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))==0);
        if(fault==1)assert(send(fd,"x",1,0)==1);
        if(fault!=2)assert(shutdown(fd,SHUT_WR)==0);
        unsigned char bytes[528];auto n=recv(fd,bytes,sizeof(bytes),0);
        if(!fault){assert(n==528);auto state=sprite_policy::decode_state(bytes,n,123,0,100000001,30000000);assert(state.sequence==1 && state.position[0]==.25);}
        close(fd);
    };
    {sprite_policy::ReadonlyStateServer server(path,123);bool collision=false;try{sprite_policy::ReadonlyStateServer duplicate(path,123);}catch(...){collision=true;}assert(collision);
     std::thread peer(client,0);server.accept_peer();sprite_policy::ProtectionVector q{},v{};q[0]=.25;
     bool stale=false;try{server.publish(q,v,1,100000001);}catch(...){stale=true;}assert(stale);assert(server.frames()==0);
     server.publish(q,v,100000000,100000001);assert(server.frames()==1);peer.join();}
    for(int fault:{1,2}){auto server=std::make_unique<sprite_policy::ReadonlyStateServer>(path,123);std::thread peer(client,fault);bool blocked=false;try{server->accept_peer();}catch(...){blocked=true;}assert(blocked);assert(server->frames()==0);
     if(fault==1)assert(server->reverse_bytes()==1);
     server.reset();peer.join();}
    assert(rmdir(d)==0);
}
