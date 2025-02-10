#pragma once

#include <string>
#include <map>
#include <type_traits>

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/time/real_time_clock.h"

#include "daikin_ekhhe_const.h"

namespace esphome {
namespace daikin_ekkhe {

class DaikinEkhheComponent;  // Forward declaration


class DaikinEkhheNumber : public number::Number {
 public:
    void control(float value) override;
    void set_parent(DaikinEkhheComponent *parent) { this->parent_ = parent; }
    // Needed so we can set this from python and then reference it in the control function
    void set_internal_id(const std::string &id) { this->internal_id_ = id; }

  private:
    DaikinEkhheComponent *parent_;
    std::string internal_id_;
};

class DaikinEkhheSelect : public select::Select, public Component {
 public:
  void control(const std::string &value) override;
  void set_select_mappings(std::map<std::string, int> mappings) {
    this->select_mappings_ = std::move(mappings);
  }
  // this stores the number to read/write for each select option
  std::map<std::string, int> get_select_mappings() {
      return this->select_mappings_;
  }
  void set_parent(DaikinEkhheComponent *parent) { this->parent_ = parent; }
  void set_internal_id(const std::string &id) { this->internal_id_ = id; }

  private:
   std::map<std::string, int> select_mappings_; 
   DaikinEkhheComponent *parent_;
   std::string internal_id_;
};

class DaikinEkhheComponent : public Component, public uart::UARTDevice {
 public:

  DaikinEkhheComponent() = default;
  enum EkhheError {
    EKHHE_ERROR_NONE,

    // from library
    EKHHE_ERROR_PACKET_SIZE, 
    EKHHE_ERROR_BUFFER_EMPTY,
    EKHHE_ERROR_CHECKSUM,
    EKHHE_ERROR_PACKET_END_CODE_MISSMATCH,
  };

  struct EkhheReading {
    uint16_t low_water_temp_probe;
  };

  // ========== INTERNAL METHODS ==========
  void setup() override;
  void loop() override;
  void update();
  void dump_config() override;
  void on_shutdown();
  void set_update_interval(int interval_ms);

  // Methods to register sensors, binary sensors, and numbers
  void register_sensor(const std::string &sensor_name, esphome::sensor::Sensor *sensor);
  void register_binary_sensor(const std::string &sensor_name, esphome::binary_sensor::BinarySensor *binary_sensor);
  void register_number(const std::string &number_name, esphome::number::Number *number);
  void register_select(const std::string &select_name, select::Select *select);
  void register_timestamp_sensor(esphome::text_sensor::TextSensor *sensor);

  // Methods to update values dynamically (only for registered components)
  void set_sensor_value(const std::string &sensor_name, float value);
  void set_binary_sensor_value(const std::string &sensor_name, bool value);
  void set_number_value(const std::string &number_name, float value);
  void set_select_value(const std::string &select_name, int value);
  void update_timestamp(uint8_t hour, uint8_t minute);

  // Allow UART command sending for Number/Select control
  void send_uart_cc_command(uint8_t index, uint8_t value);


  enum EkkheDDPacket {
    DD_PACKET_START_IDX = 0,
    DD_PACKET_B_IDX     = 6,
    DD_PACKET_A_IDX     = 7,
    DD_PACKET_C_IDX     = 8,
    DD_PACKET_D_IDX     = 10,
    DD_PACKET_E_IDX     = 11,
    DD_PACKET_F_IDX     = 12,
    DD_PACKET_G_IDX     = 13,
    DD_PACKET_H_IDX     = 14,
    DD_PACKET_I_IDX     = 18,
    DD_PACKET_DIG_IDX   = 21,
    DD_PACKET_END       = 40,
    DD_PACKET_SIZE      = 41,
  };

  enum EkhheD2Packet {
    D2_PACKET_START_IDX = 0,
    D2_PACKET_MASK1_IDX = 1,
    D2_PACKET_MASK2_IDX = 2,
    D2_PACKET_MODE_IDX  = 3,
    D2_PACKET_P4_IDX    = 4,
    D2_PACKET_P7_IDX    = 5,
    D2_PACKET_P10_IDX   = 6,
    D2_PACKET_P2_IDX    = 7,
    D2_PACKET_P29_IDX   = 9,
    D2_PACKET_P31_IDX   = 10,
    D2_PACKET_P8_IDX    = 11,
    D2_PACKET_P9_IDX    = 12,
    D2_PACKET_ECO_TTARGET_IDX       = 13,
    D2_PACKET_AUTO_TTARGET_IDX      = 14,
    D2_PACKET_BOOST_TTGARGET_IDX    = 15,
    D2_PACKET_ELECTRIC_TTARGET_IDX  = 16,
    D2_PACKET_P1_IDX    = 20,
    D2_PACKET_P32_IDX   = 21,
    D2_PACKET_P3_IDX    = 22,
    D2_PACKET_P30_IDX   = 23,
    D2_PACKET_P25_IDX   = 24,
    D2_PACKET_P26_IDX   = 25,
    D2_PACKET_P27_IDX   = 26,
    D2_PACKET_P28_IDX   = 27,
    D2_PACKET_P12_IDX   = 28,
    D2_PACKET_P14_IDX   = 29,
    D2_PACKET_P24_IDX   = 30,
    D2_PACKET_P16_IDX   = 31,
    D2_PACKET_P23_IDX   = 32,
    D2_PACKET_P17_IDX   = 33, 
    D2_PACKET_P18_IDX   = 34,
    D2_PACKET_P19_IDX   = 35,
    D2_PACKET_P20_IDX   = 36,
    D2_PACKET_P21_IDX   = 37,
    D2_PACKET_P22_IDX   = 38,
    D2_PACKET_P34_IDX   = 39,
    D2_PACKET_P37_IDX   = 40,
    D2_PACKET_P38_IDX   = 41,
    D2_PACKET_P40_IDX   = 42,
    D2_PACKET_P36_IDX   = 43,
    D2_PACKET_P35_IDX   = 44,
    D2_PACKET_P41_IDX   = 45,
    D2_PACKET_P42_IDX   = 46,
    D2_PACKET_P43_IDX   = 47,
    D2_PACKET_P44_IDX   = 48,
    D2_PACKET_P45_IDX   = 49,
    D2_PACKET_P46_IDX   = 50,
    D2_PACKET_HOUR_IDX  = 56,
    D2_PACKET_MIN_IDX   = 57,
    D2_PACKET_P47_IDX   = 59,
    D2_PACKET_P48_IDX   = 60,
    D2_PACKET_P49_IDX   = 61,
    D2_PACKET_P50_IDX   = 62,
    D2_PACKET_P51_IDX   = 63,
    D2_PACKET_P52_IDX   = 64,
    D2_PACKET_END       = 70, 
    D2_PACKET_SIZE      = 71,
  };

  enum EkhheD4Packet {
    D4_PACKET_START_IDX = 0,
    D4_PACKET_END       = 50,
    D4_PACKET_SIZE      = 51,
  };

  enum EkhheC1Packet {
    C1_PACKET_START_IDX = 0,
    C1_PACKET_END       = 50,
    C1_PACKET_SIZE      = 51,
  };

  enum EkhheCCPacket {
    CC_PACKET_START_IDX = 0,
    CC_PACKET_MASK1_IDX = 1,
    CC_PACKET_MASK2_IDX = 2,
    CC_PACKET_MODE_IDX  = 3,
    CC_PACKET_P4_IDX    = 4,
    CC_PACKET_P7_IDX    = 5,
    CC_PACKET_P10_IDX   = 6,
    CC_PACKET_P2_IDX    = 7, 
    CC_PACKET_P29_IDX   = 9,
    CC_PACKET_P31_IDX   = 10,
    CC_PACKET_P8_IDX    = 11,
    CC_PACKET_P9_IDX    = 12,
    CC_PACKET_ECO_TTARGET_IDX       = 13,
    CC_PACKET_AUTO_TTARGET_IDX      = 14,
    CC_PACKET_BOOST_TTGARGET_IDX    = 15,
    CC_PACKET_ELECTRIC_TTARGET_IDX  = 16,
    CC_PACKET_P1_IDX    = 20, 
    CC_PACKET_P32_IDX   = 21,
    CC_PACKET_P3_IDX    = 22, 
    CC_PACKET_P30_IDX   = 23,
    CC_PACKET_P25_IDX   = 24,
    CC_PACKET_P26_IDX   = 25,
    CC_PACKET_P27_IDX   = 26,
    CC_PACKET_P28_IDX   = 27,
    CC_PACKET_P12_IDX   = 28,
    CC_PACKET_P14_IDX   = 29,
    CC_PACKET_P24_IDX   = 30,
    CC_PACKET_P16_IDX   = 31,
    CC_PACKET_P23_IDX   = 32,
    CC_PACKET_P17_IDX   = 33, 
    CC_PACKET_P18_IDX   = 34,
    CC_PACKET_P19_IDX   = 35,
    CC_PACKET_P20_IDX   = 36,
    CC_PACKET_P21_IDX   = 37,
    CC_PACKET_P22_IDX   = 38,
    CC_PACKET_P34_IDX   = 39,
    CC_PACKET_P37_IDX   = 40,
    CC_PACKET_P38_IDX   = 41,
    CC_PACKET_P40_IDX   = 42,
    CC_PACKET_P36_IDX   = 43,
    CC_PACKET_P35_IDX   = 44,
    CC_PACKET_P41_IDX   = 45,
    CC_PACKET_P42_IDX   = 46,
    CC_PACKET_P43_IDX   = 47,
    CC_PACKET_P44_IDX   = 48,
    CC_PACKET_P45_IDX   = 49,
    CC_PACKET_P46_IDX   = 50,
    CC_PACKET_HOUR_IDX  = 57,
    CC_PACKET_MIN_IDX   = 58,
    CC_PACKET_P47_IDX   = 60, 
    CC_PACKET_P48_IDX   = 61, 
    CC_PACKET_P49_IDX   = 62, 
    CC_PACKET_P50_IDX   = 63, 
    CC_PACKET_P51_IDX   = 64, 
    CC_PACKET_P52_IDX   = 65,
    CC_PACKET_P54_IDX   = 60,
    CC_PACKET_END       = 70, 
    CC_PACKET_SIZE      = 71,
  };

  // This is the TX/control packet
  enum EkhheCDPacket {
    CD_PACKET_START_IDX = 0,
    CD_PACKET_END       = 70,
    CD_PACKET_SIZE      = 71,
  };

 private:
  // variables for sensors etc.
  std::map<std::string, esphome::sensor::Sensor *> sensors_;
  std::map<std::string, esphome::binary_sensor::BinarySensor *> binary_sensors_;
  std::map<std::string, esphome::number::Number *> numbers_;
  std::map<std::string, DaikinEkhheSelect *> selects_;
  text_sensor::TextSensor *timestamp_sensor_ = nullptr;
  esphome::time::RealTimeClock *clock;

  // UART Processing
  uint8_t ekhhe_checksum(const std::vector<uint8_t>& data_bytes);

  void parse_dd_packet(std::vector<uint8_t> buffer);
  void parse_d2_packet(std::vector<uint8_t> buffer);
  void parse_d4_packet(std::vector<uint8_t> buffer);
  void parse_c1_packet(std::vector<uint8_t> buffer);
  void parse_cc_packet(std::vector<uint8_t> buffer);
  void print_buffer();
  void start_uart_cycle();
  void process_packet_set();
  bool packet_set_complete();
  void store_latest_packet(uint8_t byte);

  std::vector<uint8_t> buffer_;  // Stores incoming UART bytes
  std::vector<uint8_t> last_d2_packet_;
  std::vector<uint8_t> last_dd_packet_;
  std::vector<uint8_t> last_cc_packet_;  // Always store CC for sending commands
  std::vector<uint8_t> last_c1_packet_;
  std::vector<uint8_t> last_d4_packet_;
  std::map<uint8_t, std::vector<uint8_t>> latest_packets_;

  uint8_t expected_length_ = 0;  // Expected packet length
  bool receiving_ = false;       // If we're currently receiving a packet
  bool uart_active_ = false;
  bool processing_updates_ = false;

  // Cycle management
  unsigned long last_process_time_ = 0;
  unsigned long update_interval_ = 10000;
};

using namespace daikin_ekhhe;
static const std::map<std::string, uint8_t> NUMBER_PARAM_INDEX = {
   {P1_LOW_WAT_PROBE_HYST,   DaikinEkhheComponent::CC_PACKET_P1_IDX},
   {P2_HEAT_ON_DELAY,        DaikinEkhheComponent::CC_PACKET_P2_IDX},
   {P3_ANTL_SET_T,           DaikinEkhheComponent::CC_PACKET_P3_IDX},
};

}  // namespace daikin_ekkhe
}  // namespace esphome