#pragma once
#include "protocol.h"
#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/select/select.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome { namespace gree_yunyi2 {
class GreeYunyi2;
class GreeSelect : public select::Select {
 public:
  void set_parent(GreeYunyi2 *p) { parent_=p; }
  void set_kind(uint8_t k) { kind_=k; }
 protected:
  void control(const std::string &value) override;
  GreeYunyi2 *parent_{nullptr};
  uint8_t kind_{0};
};
class GreeYunyi2 : public Component, public climate::Climate, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void set_poll_interval(uint32_t v) { poll_interval_=v; }
  void set_aux_heat_write_enabled(bool v) { aux_write_=v; }
  void set_report_current_temperature(bool v) { report_temperature_=v; }
  void set_packet_log(bool v) { packet_log_=v; }
  void set_select(uint8_t k, GreeSelect *v) { if(k<5) selects_[k]=v; }
  void set_connection(binary_sensor::BinarySensor *v) { connection_=v; }
  void select_control(uint8_t kind, size_t option);
  void send_test_hex(const std::string &hex);
  void request_status();
  void reinitialize() { restart_link_(); }
 protected:
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;
  void receive_(const protocol::Frame &f);
  void transmit_(protocol::Frame f);
  void log_frame_(const char *direction, const protocol::Frame &f);
  void publish_status_();
  void restart_link_();
  bool can_control_();
  void queue_(protocol::Frame f);
  void set_online_(bool online);
  enum class Stage { HELLO, IDENTIFY, CAPABILITY, FIRST_STATUS, READY };
  Stage stage_{Stage::HELLO};
  protocol::Parser parser_;
  protocol::Frame status_, pending_, sent_control_;
  GreeSelect *selects_[5]{nullptr,nullptr,nullptr,nullptr,nullptr};
  binary_sensor::BinarySensor *connection_{nullptr};
  uint32_t poll_interval_{1000}, boot_ms_{0}, last_tx_{0}, last_rx_byte_{0}, last_status_{0}, last_heartbeat_{0}, last_publish_{0};
  uint8_t attempts_{0};
  bool stage_sent_{false}, online_{false}, waiting_{false}, clear_after_control_{false};
  bool aux_write_{false}, report_temperature_{false}, packet_log_{true};
};
}}
