#pragma once

#include <string>
#include <map>
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "daikin_ekhhe_const.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
//#include "esphome/components/switch_/switch.h"
//#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"


namespace esphome {
namespace daikin_ekkhe {

class DaikinEkhheNumber : public number::Number {
 public:
  void control(float value) override;
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
  // Nothing really public.

  // ========== INTERNAL METHODS ==========
  void setup() override;
  void loop() override;
  void update();
  void dump_config() override;
  void on_shutdown();

  // Methods to register sensors, binary sensors, and numbers
  void register_sensor(const std::string &sensor_name, esphome::sensor::Sensor *sensor);
  void register_binary_sensor(const std::string &sensor_name, esphome::binary_sensor::BinarySensor *binary_sensor);
  void register_number(const std::string &number_name, esphome::number::Number *number);
  //void register_switch(int switch_id, esphome::switch::Switch *switch);

  // Methods to update values dynamically (only for registered components)
  void set_sensor_value(const std::string &sensor_name, float value);
  void set_binary_sensor_value(const std::string &sensor_name, bool value);
  void set_number_value(const std::string &number_name, float value);
  //void set_switch_state(int switch_id, bool state);

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
    D2_PACKET_POWER_IDX = 1,
    D2_PACKET_MODE_IDX  = 3,
    D2_PACKET_P4_IDX    = 4,  // CONFIRMED
    D2_PACKET_P2_IDX    = 7,  // CONFIRMED
    D2_PACKET_TTEMP_IDX = 14, // CONFIRMED
    D2_PACKET_P1_IDX    = 20, // CONFIRMED
    D2_PACKET_P3_IDX    = 22, // CONFIRMED
    D2_PACKET_P24_IDX   = 30,
    D2_PACKET_P16_IDX   = 31,
    D2_PACKET_P23_IDX   = 32,
    D2_PACKET_P18_IDX   = 34, // TBC
    D2_PACKET_P19_IDX   = 35, // TBC
    D2_PACKET_P20_IDX   = 36, // TBC
    D2_PACKET_P22_IDX   = 38,  // TBC
    D2_PACKET_P36_IDX   = 43, // TBC
    D2_PACKET_P47_IDX   = 59, // TBC
    D2_PACKET_P48_IDX   = 60, // TBC
    D2_PACKET_P49_IDX   = 61, // TBC
    D2_PACKET_P50_IDX   = 62, // TBC
    D2_PACKET_P51_IDX   = 63, // TBC
    D2_PACKET_P52_IDX   = 64, // TBC 
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
    CC_PACKET_END       = 70,
    CC_PACKET_SIZE      = 71,
  };


 private:
  std::map<std::string, esphome::sensor::Sensor *> sensors_;
  std::map<std::string, esphome::binary_sensor::BinarySensor *> binary_sensors_;
  std::map<std::string, esphome::number::Number *> numbers_;
  //std::map<int, esphome::switch::Switch  *> switches_;


  std::vector<uint8_t> buffer_;  // Stores incoming UART bytes
  uint8_t expected_length_ = 0;  // Expected packet length
  bool receiving_ = false;       // If we're currently receiving a packet


  DaikinEkhheComponent::EkhheError read_packet_();
  uint8_t ekhhe_checksum(const std::vector<uint8_t>& data_bytes);
  void parse_dd_packet();
  void parse_d2_packet();
  void parse_d4_packet();
  void parse_c1_packet();
  void parse_cc_packet();
  void print_buffer();



  // UART Processing
  DaikinEkhheComponent::EkhheError process_uart_buffer();
  void send_uart_command(int number_id, float value);
  void send_uart_switch_command(int switch_id, bool state);

};

}  // namespace daikin_ekkhe
}  // namespace esphome