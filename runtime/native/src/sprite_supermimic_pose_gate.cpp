// File-only run002 position/speed admission diagnostic. No transport or motor API.
#include "run002_pose_limits.hpp"
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Vector=std::array<double,31>;
struct Verdict {int motor=-1;const char* reason="accepted";};
Verdict check(const Vector& q,const Vector& v){
    for(std::size_t i=0;i<31;++i){
        const auto& limit=sprite_run002::limits[i];
        if(!std::isfinite(q[i])||!std::isfinite(v[i]))return {static_cast<int>(i),"nonfinite_state"};
        if(q[i]<limit.lower||q[i]>limit.upper)return {static_cast<int>(i),"hard_position"};
        if(std::abs(v[i])>limit.speed)return {static_cast<int>(i),"motor_speed"};
    }
    return {};
}
void require(bool ok){if(!ok)throw std::runtime_error("pose gate self-test failed");}
void self_test(){
    Vector q{},v{};
    for(std::size_t i=0;i<31;++i){const auto& l=sprite_run002::limits[i];q[i]=(l.lower+l.upper)/2;}
    require(check(q,v).motor<0);
    for(std::size_t i=0;i<31;++i){
        const auto& l=sprite_run002::limits[i];const double original=q[i];
        for(double edge:{l.lower,l.upper}){q[i]=edge;require(check(q,v).motor<0);}
        q[i]=std::nextafter(l.upper,std::numeric_limits<double>::infinity());require(check(q,v).motor==static_cast<int>(i));
        q[i]=std::nextafter(l.lower,-std::numeric_limits<double>::infinity());require(check(q,v).motor==static_cast<int>(i));
        q[i]=original;v[i]=l.speed;require(check(q,v).motor<0);
        v[i]=std::nextafter(l.speed,std::numeric_limits<double>::infinity());require(check(q,v).motor==static_cast<int>(i));
        v[i]=0;q[i]=std::numeric_limits<double>::quiet_NaN();require(check(q,v).motor==static_cast<int>(i));q[i]=original;
    }
    std::cout<<"RUN002_POSE_LIMIT_BOUNDARIES_PASS motors=31 hardware_tx=0\n";
}
}

int main(int argc,char** argv){try{
    if(argc==2&&std::string(argv[1])=="--self-test"){self_test();return 0;}
    if(argc!=2)throw std::runtime_error("Usage: sprite_supermimic_pose_gate STATE_BIN | --self-test (files only)");
    const std::uint32_t endian=1;if(*reinterpret_cast<const char*>(&endian)!=1)throw std::runtime_error("little endian required");
    std::ifstream file(argv[1],std::ios::binary);char magic[8]{};std::uint64_t hash=0;Vector q{},v{};
    file.read(magic,8);file.read(reinterpret_cast<char*>(&hash),sizeof(hash));
    file.read(reinterpret_cast<char*>(q.data()),sizeof(q));file.read(reinterpret_cast<char*>(v.data()),sizeof(v));
    if(!file||std::memcmp(magic,"SPPOSE01",8)||hash!=sprite_run002::name_hash||file.peek()!=std::char_traits<char>::eof())
        throw std::runtime_error("invalid pose record/motor order/trailing bytes");
    const auto verdict=check(q,v);
    std::cout.precision(17);
    std::cout<<"{\"admitted\":"<<(verdict.motor<0?"true":"false")<<",\"reason\":\""<<verdict.reason
        <<"\",\"hardware_sha256\":\""<<sprite_run002::hardware_sha256<<"\",\"policy_sha256\":\""<<sprite_run002::policy_sha256
        <<"\",\"hardware_tx\":0,\"motor_enable_attempts\":0,\"hardware_qualified\":false";
    if(verdict.motor>=0){const auto i=verdict.motor;const auto& l=sprite_run002::limits[i];
        std::cout<<",\"motor\":\""<<l.name<<"\",\"lower_rad\":"<<l.lower<<",\"upper_rad\":"<<l.upper;
        if(std::isfinite(q[i]))std::cout<<",\"position_rad\":"<<q[i];
        if(std::isfinite(v[i]))std::cout<<",\"speed_rad_s\":"<<v[i];
    }
    std::cout<<",\"scope\":\"Frozen recorded position/speed check only; no freshness, IMU, torque or dynamic admission\"}\n";
    return verdict.motor<0?0:3;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
