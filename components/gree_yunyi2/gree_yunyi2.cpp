#include "gree_yunyi2.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cstdio>

namespace esphome { namespace gree_yunyi2 {
static const char *const TAG="gree_yunyi2";
static const char *const FAN_NAMES[]={"Auto","1","2","3","4","5","Quiet","Turbo"};
void GreeSelect::control(const std::string &value) {
  auto i=this->index_of(value);
  if(parent_ && i.has_value()) parent_->select_control(kind_,*i);
}
void GreeYunyi2::setup() {
  boot_ms_=millis(); last_tx_=boot_ms_; last_rx_byte_=boot_ms_;
  this->set_supported_custom_fan_modes(FAN_NAMES);
  if(connection_) connection_->publish_state(false);
  this->status_set_warning();
}
climate::ClimateTraits GreeYunyi2::traits() {
  climate::ClimateTraits t;
  if(report_temperature_) t.set_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  t.set_supported_modes({climate::CLIMATE_MODE_OFF,climate::CLIMATE_MODE_AUTO,climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,climate::CLIMATE_MODE_DRY,climate::CLIMATE_MODE_FAN_ONLY});
  t.set_supported_swing_modes({climate::CLIMATE_SWING_OFF,climate::CLIMATE_SWING_VERTICAL,
    climate::CLIMATE_SWING_HORIZONTAL,climate::CLIMATE_SWING_BOTH});
  t.set_visual_min_temperature(16); t.set_visual_max_temperature(30); t.set_visual_temperature_step(1);
  return t;
}
void GreeYunyi2::dump_config() {
  ESP_LOGCONFIG(TAG,"Gree Yunyi2 v0.1.0: captured 50-byte variant; UART 4800 8E1");
  ESP_LOGCONFIG(TAG,"Poll: %u ms; aux writes: %s; inferred room temperature: %s",unsigned(poll_interval_),
    aux_write_?"enabled (experimental)":"disabled",report_temperature_?"enabled":"disabled");
}
void GreeYunyi2::log_frame_(const char *direction,const protocol::Frame &f) {
  if(!packet_log_) return;
  std::string hex; char buf[4]; hex.reserve(f.size()*3);
  for(uint8_t v:f) { snprintf(buf,sizeof(buf),"%02X ",v); hex+=buf; }
  ESP_LOGD(TAG,"%s %s",direction,hex.c_str());
}
void GreeYunyi2::transmit_(protocol::Frame f) {
  if(f.empty()) return;
  protocol::checksum(f); log_frame_("TX",f); this->write_array(f);
  last_tx_=millis();
}
void GreeYunyi2::set_online_(bool value) {
  if(online_==value) return;
  online_=value;
  if(connection_) connection_->publish_state(value);
  if(value) { this->status_clear_warning(); ESP_LOGI(TAG,"AC status received; controls available"); }
  else { this->status_set_warning(); ESP_LOGW(TAG,"AC link offline; commands discarded"); }
}
void GreeYunyi2::restart_link_() {
  set_online_(false); stage_=Stage::HELLO; stage_sent_=false; attempts_=0;
  waiting_=false; clear_after_control_=false;
  status_.clear(); pending_.clear(); sent_control_.clear(); parser_.clear();
  boot_ms_=millis(); last_tx_=boot_ms_;
}
void GreeYunyi2::receive_(const protocol::Frame &f) {
  log_frame_("RX",f);
  if(stage_==Stage::HELLO && f[3]==0x32) {
    stage_=Stage::IDENTIFY; stage_sent_=false; attempts_=0; return;
  }
  if(stage_==Stage::IDENTIFY && f[3]==0x44) {
    stage_=Stage::CAPABILITY; stage_sent_=false; attempts_=0; return;
  }
  if(!protocol::is_status(f)) return;  // 0x33 is telemetry, not a settings snapshot.
  bool changed=status_!=f;
  if(!sent_control_.empty()) {
    // Compare the known settings fields, not device-only event/telemetry bytes.
    bool same=true;
    for(size_t i: {size_t(8),size_t(9),size_t(10),size_t(11),size_t(12),size_t(13),size_t(15),size_t(20),size_t(22)})
      if(sent_control_[i]!=f[i]) same=false;
    if(!same) ESP_LOGW(TAG,"AC reported adjusted/rejected settings; using returned state, no automatic retry");
    else ESP_LOGD(TAG,"AC reported requested settings");
    sent_control_.clear();
  }
  status_=f; last_status_=millis(); waiting_=false;
  if(stage_!=Stage::READY) { stage_=Stage::READY; last_heartbeat_=millis(); changed=true; }
  set_online_(true);
  if(changed || millis()-last_publish_>=30000) publish_status_();
}
void GreeYunyi2::loop() {
  uint32_t now=millis();
  if(now-last_rx_byte_>300) parser_.clear();
  // Bounded read work keeps networking/OTA responsive even on a noisy input.
  for(size_t count=0;count<256 && this->available();++count) {
    uint8_t v;
    if(!this->read_byte(&v)) break;
    last_rx_byte_=now;
    parser_.feed(v,[this](const protocol::Frame &f){ receive_(f); });
  }
  now=millis();
  if(stage_==Stage::READY && now-last_status_>10000) { restart_link_(); return; }
  if(stage_!=Stage::READY) {
    if(now-boot_ms_<1000) return;
    if(stage_==Stage::HELLO || stage_==Stage::IDENTIFY) {
      if(now-last_tx_<200 || (stage_sent_ && now-last_tx_<1000)) return;
      if(attempts_>=3) {
        // Some revisions start reporting without the expected handshake reply.
        ESP_LOGW(TAG,"Handshake response missing, trying status request");
        stage_=Stage::FIRST_STATUS; stage_sent_=false; attempts_=0; return;
      }
      transmit_(stage_==Stage::HELLO?protocol::hello():protocol::identify());
      stage_sent_=true; ++attempts_; return;
    }
    if(stage_==Stage::CAPABILITY) {
      if(!stage_sent_ && now-last_tx_>=200) { transmit_(protocol::capability()); stage_sent_=true; return; }
      if(stage_sent_ && now-last_tx_>=1000) { stage_=Stage::FIRST_STATUS; stage_sent_=false; attempts_=0; }
      return;
    }
    if(stage_==Stage::FIRST_STATUS && now-last_tx_>=1000) {
      if(attempts_>=5) { restart_link_(); return; }
      transmit_(protocol::heartbeat(stage_sent_?1:4)); stage_sent_=true; ++attempts_;
    }
    return;
  }
  if(waiting_) {
    if(now-last_tx_<1000) return;
    ESP_LOGW(TAG,"No status response within 1s; write is NOT retried");
    waiting_=false; sent_control_.clear();
  }
  if(now-last_tx_<250) return;
  if(!pending_.empty()) {
    // User commands are single-shot. No restoration of stored climate state.
    sent_control_=pending_; transmit_(pending_); pending_.clear();
    waiting_=true; clear_after_control_=true; return;
  }
  if(clear_after_control_) {
    transmit_(protocol::parameters(status_)); clear_after_control_=false; waiting_=true; return;
  }
  if(now-last_heartbeat_>=15000) {
    transmit_(protocol::heartbeat(1)); last_heartbeat_=now; waiting_=true; return;
  }
  if(now-last_tx_>=poll_interval_) { transmit_(protocol::parameters(status_)); waiting_=true; }
}
bool GreeYunyi2::can_control_() {
  if(!online_ || stage_!=Stage::READY || millis()-last_status_>5000 || status_.empty()) {
    ESP_LOGW(TAG,"Control ignored: no recent AC status"); return false;
  }
  if(!pending_.empty() || !sent_control_.empty() || clear_after_control_) {
    ESP_LOGW(TAG,"Control ignored: previous command still pending; wait for state update"); return false;
  }
  return true;
}
void GreeYunyi2::queue_(protocol::Frame f) {
  if(f.empty()) return;
  f[7]=0xAF; protocol::checksum(f); pending_=std::move(f);
}
void GreeYunyi2::control(const climate::ClimateCall &call) {
  if(!can_control_()) return;
  auto f=protocol::parameters(status_,true); bool touched=false;
  if(call.get_mode().has_value()) {
    int m;
    switch(*call.get_mode()) {
      case climate::CLIMATE_MODE_OFF: m=-1; break;
      case climate::CLIMATE_MODE_AUTO: m=0; break;
      case climate::CLIMATE_MODE_COOL: m=1; break;
      case climate::CLIMATE_MODE_DRY: m=2; break;
      case climate::CLIMATE_MODE_FAN_ONLY: m=3; break;
      case climate::CLIMATE_MODE_HEAT: m=4; break;
      default: return;
    }
    protocol::set_mode(f,m); touched=true;
  }
  if(call.get_target_temperature().has_value()) {
    if(!protocol::set_temperature(f,*call.get_target_temperature())) { ESP_LOGW(TAG,"Temperature must be 16..30 C"); return; }
    touched=true;
  }
  if(call.has_custom_fan_mode()) {
    auto name=call.get_custom_fan_mode(); bool found=false;
    for(int i=0;i<8;++i) if(name==FAN_NAMES[i]) { protocol::set_fan(f,i); found=true; }
    if(!found) return;
    touched=true;
  }
  if(call.get_swing_mode().has_value()) {
    uint8_t v=0;
    switch(*call.get_swing_mode()) {
      case climate::CLIMATE_SWING_VERTICAL: v=0x10; break;
      case climate::CLIMATE_SWING_HORIZONTAL: v=0x01; break;
      case climate::CLIMATE_SWING_BOTH: v=0x11; break;
      case climate::CLIMATE_SWING_OFF: v=0; break;
      default: return;
    }
    protocol::masked(f,12,0xF7,v); touched=true;
  }
  if(touched) queue_(f);
}
void GreeYunyi2::select_control(uint8_t kind,size_t option) {
  if(kind>=5 || !can_control_()) return;
  auto f=protocol::parameters(status_,true);
  switch(kind) {
    case 0:
      if(!aux_write_) { ESP_LOGW(TAG,"Aux writes disabled; set aux_heat_write_enabled: true for a deliberate test"); return; }
      if(!protocol::set_aux(f,int(option))) { ESP_LOGW(TAG,"Aux selection only writable in retained HEAT mode, options ON/OFF/blank"); return; }
      break;
    case 1: if(option>6) return; protocol::masked(f,12,0xF0,uint8_t(option)<<4); break;
    case 2: if(option>6) return; protocol::masked(f,12,0x07,uint8_t(option)); break;
    case 3: if(option>1) return; protocol::masked(f,10,0x02,option?2:0); break;
    case 4: if(option>1) return; protocol::masked(f,8,0x08,option?8:0); break;
  }
  queue_(f);
}
void GreeYunyi2::send_test_hex(const std::string &hex) {
  // Space-separated two-digit hex. Validate everything BEFORE any UART write.
  protocol::Frame f; int high=-1;
  for(unsigned char c:hex) {
    if(c==' ' || c=='\t' || c=='\n' || c=='\r') {
      if(high!=-1) { ESP_LOGW(TAG,"HEX rejected: each byte needs two digits"); return; }
      continue;
    }
    int v=c>='0'&&c<='9'?c-'0':c>='A'&&c<='F'?c-'A'+10:c>='a'&&c<='f'?c-'a'+10:-1;
    if(v<0) { ESP_LOGW(TAG,"HEX rejected: use hex digits/spaces, no 0x prefixes"); return; }
    if(high<0) high=v; else { f.push_back(uint8_t(high*16+v)); high=-1; }
    if(f.size()>96) { ESP_LOGW(TAG,"HEX rejected: frame too long"); return; }
  }
  if(high>=0 || !protocol::valid(f)) { ESP_LOGW(TAG,"HEX rejected: header, length or checksum invalid"); return; }
  if(millis()-last_tx_<250 || waiting_ || !pending_.empty() || !sent_control_.empty() || clear_after_control_) {
    ESP_LOGW(TAG,"Test not sent: UART transaction busy; press again after state update"); return;
  }
  ESP_LOGI(TAG,"Sending ONE manual test frame (%u bytes)",unsigned(f.size()));
  transmit_(f);
  if(stage_==Stage::READY) {
    waiting_=true;
    if(f.size()==50 && f[3]==1 && f[7]==0xAF) { sent_control_=f; clear_after_control_=true; }
  }
}
void GreeYunyi2::request_status() {
  if(millis()-last_tx_<250 || waiting_ || !pending_.empty() || !sent_control_.empty() || clear_after_control_) {
    ESP_LOGW(TAG,"Status request not sent: busy; retry shortly"); return;
  }
  transmit_(protocol::heartbeat(1));
  if(stage_==Stage::READY) waiting_=true;
}
void GreeYunyi2::publish_status_() {
  if(status_.empty()) return;
  const auto &f=status_;
  int m=(f[8]>>4)&7;
  static const climate::ClimateMode modes[]={climate::CLIMATE_MODE_AUTO,climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_DRY,climate::CLIMATE_MODE_FAN_ONLY,climate::CLIMATE_MODE_HEAT};
  this->mode=(f[8]&0x80) && m<=4?modes[m]:climate::CLIMATE_MODE_OFF;
  this->target_temperature=16+(f[9]>>4);
  if(report_temperature_) {
    int c=int(f[46])-40;
    this->current_temperature=(c>=-10 && c<=60)?float(c):NAN;
  }
  int fan=protocol::fan(f);
  if(fan>=0) this->set_custom_fan_mode_(FAN_NAMES[fan]);
  bool v=(f[12]>>4)==1, h=(f[12]&7)==1;
  this->swing_mode=v?(h?climate::CLIMATE_SWING_BOTH:climate::CLIMATE_SWING_VERTICAL):
    (h?climate::CLIMATE_SWING_HORIZONTAL:climate::CLIMATE_SWING_OFF);
  auto publish=[this](size_t kind,size_t value) {
    auto *s=selects_[kind];
    if(s && s->has_index(value) && (!s->active_index().has_value() || *s->active_index()!=value)) s->publish_state(value);
  };
  publish(0,size_t(protocol::aux(f))); publish(1,f[12]>>4); publish(2,f[12]&7);
  publish(3,(f[10]&2)?1:0); publish(4,(f[8]&8)?1:0);
  last_publish_=millis(); this->publish_state();
}
}}
