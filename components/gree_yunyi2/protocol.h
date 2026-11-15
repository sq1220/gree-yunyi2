#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace esphome { namespace gree_yunyi2 { namespace protocol {
using Frame = std::vector<uint8_t>;
inline bool valid(const Frame &f) {
  if (f.size() < 5 || f[0] != 0x7E || f[1] != 0x7E || f.size() != size_t(f[2]) + 3) return false;
  uint8_t sum = 0;
  for (size_t i = 2; i + 1 < f.size(); ++i) sum += f[i];
  return sum == f.back();
}
inline void checksum(Frame &f) {
  uint8_t sum = 0;
  for (size_t i = 2; i + 1 < f.size(); ++i) sum += f[i];
  f.back() = sum;
}
inline Frame frame(uint8_t length, uint8_t command) {
  Frame f(length + 3, 0); f[0] = f[1] = 0x7E; f[2] = length; f[3] = command; checksum(f); return f;
}
inline Frame hello() {
  Frame f = {0x7E,0x7E,0x10,0x02,0,0,0,0,0,0,1,0,0x28,0x1E,0x19,0x23,0x23,0,0};
  checksum(f); return f;
}
inline Frame identify() { auto f=frame(0x0C,0x04); f[4]=7; checksum(f); return f; }
inline Frame capability() { return frame(0x34,0x0C); }
// Captured module status frame. This is NOT a verified Wi-Fi LED command.
inline Frame heartbeat(uint8_t state=1) {
  auto f=frame(0x19,0x03); f[10]=f[13]=0x3B; f[26]=state; checksum(f); return f;
}
inline bool is_status(const Frame &f) { return f.size()==50 && f[3]==0x31 && valid(f); }
inline Frame parameters(const Frame &status, bool write=false) {
  if (!is_status(status)) return {};
  auto f=frame(0x2F,0x01);
  // Preserve the captured settings region, never echo header flags, event byte 41,
  // or temperature telemetry at 46. This layout is specific to these captures.
  std::copy(status.begin()+8, status.begin()+41, f.begin()+8);
  f[7]=write ? 0xAF : 0; f[43]=2; checksum(f); return f;
}
inline void masked(Frame &f, size_t i, uint8_t mask, uint8_t value) {
  f[i]=(f[i]&uint8_t(~mask))|(value&mask);
}
inline bool set_mode(Frame &f,int mode) {
  // -1 off, 0 auto, 1 cool, 2 dry, 3 fan, 4 heat.
  if(mode < -1 || mode > 4) return false;
  if(mode == -1) masked(f,8,0x80,0);
  else masked(f,8,0xF0,uint8_t(0x80|(mode<<4)));
  return true;
}
inline bool set_temperature(Frame &f,float c) {
  if(!std::isfinite(c) || c<16 || c>30) return false;
  masked(f,9,0xF0,uint8_t(std::lround(c)-16)<<4); return true;
}
// Fan 0 auto, 1..5 captured speed codes, 6 quiet, 7 turbo.
inline bool set_fan(Frame &f,int fan) {
  if(fan<0 || fan>7) return false;
  masked(f,20,0x08,0); masked(f,10,0x01,0);
  int speed=fan;
  if(fan==6) { speed=1; masked(f,20,0x08,0x08); }
  if(fan==7) { speed=5; masked(f,10,0x01,0x01); }
  static const uint8_t coarse[]={0,1,1,2,2,3};
  masked(f,8,0x03,coarse[speed]); masked(f,22,0x0F,uint8_t(speed)); return true;
}
inline int fan(const Frame &f) {
  if(f[10]&1) return 7;
  if(f[20]&8) return 6;
  int v=f[22]&0x0F; return v<=5 ? v : -1;
}
// User-correlated HEAT ONLY: 0=ON(A), 1=OFF(B), 2=blank(C).
// Blank is deliberately not labelled AUTO. This is a setting, not heater activity.
inline bool set_aux(Frame &f,int state) {
  if(((f[8]>>4)&7)!=4 || state<0 || state>2) return false;
  masked(f,10,0x08,state==1 ? 0x08 : 0);
  masked(f,15,0x80,state==2 ? 0x80 : 0); return true;
}
inline int aux(const Frame &f) {
  if(((f[8]>>4)&7)!=4) return 3;
  bool a=f[10]&8, b=f[15]&0x80;
  return a&&b ? 4 : b ? 2 : a ? 1 : 0;
}
class Parser {
 public:
  void clear() { buffer_.clear(); }
  template<class Callback> void feed(uint8_t byte, Callback callback) {
    buffer_.push_back(byte);
    while(buffer_.size()>=2) {
      if(buffer_[0]!=0x7E || buffer_[1]!=0x7E) { buffer_.erase(buffer_.begin()); continue; }
      if(buffer_.size()<3) return;
      if(buffer_[2]<2 || buffer_[2]>93) { buffer_.erase(buffer_.begin()); continue; }
      size_t length=buffer_[2]+3;
      if(buffer_.size()<length) return;
      Frame f(buffer_.begin(),buffer_.begin()+length);
      if(!valid(f)) { ++bad_checksums; buffer_.erase(buffer_.begin()); continue; }
      buffer_.erase(buffer_.begin(),buffer_.begin()+length); callback(f);
    }
  }
  uint32_t bad_checksums{0};
 protected:
  Frame buffer_;
};
}}}
