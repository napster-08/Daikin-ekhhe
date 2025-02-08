#include "daikin_ekhhe.h"
#include "daikin_ekhhe_const.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <numeric>

namespace esphome {
namespace daikin_ekkhe {

using namespace daikin_ekhhe;

static const char *const TAG = "daikin_ekhhe.sensor";
//static const uint8_t ASCII_CR = 0x0D;
//static const uint8_t ASCII_NBSP = 0xFF;
//static const int MAX_DATA_LENGTH_BYTES = 6;

static const uint8_t DD_PACKET_START_BYTE = 0xDD;
static const uint8_t D2_PACKET_START_BYTE = 0xD2;
static const uint8_t D4_PACKET_START_BYTE = 0xD4;
static const uint8_t C1_PACKET_START_BYTE = 0xC1;
static const uint8_t CC_PACKET_START_BYTE = 0xCC;

// Packet definitions
static const std::map<uint8_t, uint8_t> PACKET_SIZES = {
    {DD_PACKET_START_BYTE, DaikinEkhheComponent::DD_PACKET_SIZE},
    {D2_PACKET_START_BYTE, DaikinEkhheComponent::D2_PACKET_SIZE},
    {D4_PACKET_START_BYTE, DaikinEkhheComponent::D4_PACKET_SIZE},
    {C1_PACKET_START_BYTE, DaikinEkhheComponent::C1_PACKET_SIZE},
    {CC_PACKET_START_BYTE, DaikinEkhheComponent::CC_PACKET_SIZE},
};

//static const uint8_t KAIDI_END_BYTE = 0x16;
//static const uint8_t KAIDI_MODE_RECEIVED = 0x00;



void DaikinEkhheComponent::setup() {
    ESP_LOGI(TAG, "Setting up Daikin EKHHE component...");
    //this->publish_state(""); 

}

void DaikinEkhheComponent::loop() {
  while (this->available()) {
    uint8_t byte = this->read();

    if (!receiving_) {
      // Check if this byte is a valid packet start byte
      if (PACKET_SIZES.find(byte) != PACKET_SIZES.end()) {
        ESP_LOGI(TAG, "Detected packet start byte: 0x%X", byte);
        buffer_.clear();
        buffer_.push_back(byte);
        expected_length_ = PACKET_SIZES.at(byte);
        receiving_ = true;
      }
    } else {
      // Receiving a packet
      buffer_.push_back(byte);

      if (buffer_.size() >= expected_length_) {
        // Packet fully received
        receiving_ = false;
        process_uart_buffer();
      }
    }
  }
}

DaikinEkhheComponent::EkhheError DaikinEkhheComponent::process_uart_buffer() {

  // TODO: Implement actual UART parsing logic to extract sensor values.
  // This is just an example.

  // Checksum check
  if (buffer_.empty()) {
    return EKHHE_ERROR_BUFFER_EMPTY;
  }

  uint8_t checksum = ekhhe_checksum(buffer_);

  if (checksum != buffer_.back()) {
    return EKHHE_ERROR_CHECKSUM;
  }

  uint8_t packet_type = buffer_[0];

  switch (packet_type) {
    case DD_PACKET_START_BYTE:
      parse_dd_packet();
      break;
    case D2_PACKET_START_BYTE:
      parse_d2_packet();
      break;
    case D4_PACKET_START_BYTE:
      parse_d4_packet();
      break;
    case C1_PACKET_START_BYTE:
      parse_c1_packet();
      break;
    case CC_PACKET_START_BYTE:
      parse_cc_packet();
      break;
  }

  return EKHHE_ERROR_NONE;
}

void DaikinEkhheComponent::print_buffer() {
  std::string hex_output;
  for (size_t i = 0; i < buffer_.size(); i++) {
    char hex_byte[6];  // Enough space for "0xXX "
    snprintf(hex_byte, sizeof(hex_byte), "0x%02X ", buffer_[i]);
    hex_output += hex_byte;

    // Print in groups of 8 for readability
    if ((i + 1) % 8 == 0) {
      hex_output += "\n";
    }
  }

  ESP_LOGV("daikin_ekhhe", "Buffer Contents:\n%s", hex_output.c_str());
}

void DaikinEkhheComponent::parse_dd_packet() {

  // update sensors
  std::map<std::string, float> sensor_values = {
      {A_LOW_WAT_T_PROBE, (int8_t)buffer_[DD_PACKET_A_IDX]},
      {B_UP_WAT_T_PROBE, (int8_t)buffer_[DD_PACKET_B_IDX]},
      {C_DEFROST_T_PROBE, (int8_t)buffer_[DD_PACKET_C_IDX]},
      {D_SUPPLY_AIR_T_PROBE, (int8_t)buffer_[DD_PACKET_D_IDX]},
      {E_EVA_INLET_T_PROBE, (int8_t)buffer_[DD_PACKET_E_IDX]},
      {F_EVA_OUTLET_T_PROBE, (int8_t)buffer_[DD_PACKET_F_IDX]},
      {G_COMP_GAS_T_PROBE, buffer_[DD_PACKET_H_IDX]},
      {H_SOLAR_T_PROBE, buffer_[DD_PACKET_H_IDX]},
      {I_EEV_STEP, buffer_[DD_PACKET_I_IDX]},
  };

  for (const auto &entry : sensor_values) {
    set_sensor_value(entry.first, entry.second);
  }

  // update binary_sensors
  std::map<std::string, bool> binary_sensor_values = {
      {DIG1_CONFIG, (bool)(buffer_[DD_PACKET_DIG_IDX] & 0x01)},
      {DIG2_CONFIG, (bool)(buffer_[DD_PACKET_DIG_IDX] & 0x02)},
      {DIG3_CONFIG, (bool)(buffer_[DD_PACKET_DIG_IDX] & 0x04)},
  };

  for (const auto &entry : binary_sensor_values) {
    DaikinEkhheComponent::set_binary_sensor_value(entry.first, entry.second);
  }

  print_buffer();
  return;
}

void DaikinEkhheComponent::parse_d2_packet() {

  // update numbers
  std::map<std::string, float> number_values = {
      {P4_ANTL_DURATION, buffer_[D2_PACKET_P4_IDX]},
      {P2_HEAT_ON_DELAY, buffer_[D2_PACKET_P2_IDX]},
      {P1_LOW_WAT_PROBE_HYST, buffer_[D2_PACKET_P1_IDX]},
      {P3_ANTL_SET_T, buffer_[D2_PACKET_P3_IDX]},
      {TARGET_TEMPERATURE, buffer_[D2_PACKET_TTEMP_IDX]},
      {P18_LOW_WAT_T_DIG1, buffer_[D2_PACKET_P18_IDX]},
      {P19_LOW_WAT_T_HYST, buffer_[D2_PACKET_P19_IDX]},
      {P20_SOL_DRAIN_THRES, buffer_[D2_PACKET_P20_IDX]},
      {P22_UP_WAT_T_EH_STOP, buffer_[D2_PACKET_P22_IDX]},
      {P36_EEV_DSH_SETPOINT, buffer_[D2_PACKET_P36_IDX]},
      {P47_MAX_INLET_T_HP, buffer_[D2_PACKET_P47_IDX]},
      {P48_MIN_INLET_T_HP, (int8_t)buffer_[D2_PACKET_P48_IDX]},
      {P49_EVA_INLET_THRES, buffer_[D2_PACKET_P49_IDX]},
      {P50_ANTIFREEZE_SET, buffer_[D2_PACKET_P50_IDX]},
      {P51_EVA_HIGH_SET, buffer_[D2_PACKET_P51_IDX]},
      {P52_EVA_LOW_SET, buffer_[D2_PACKET_P52_IDX]},
  };

  for (const auto &entry : number_values) {
    set_number_value(entry.first, entry.second);
  }

  // update selects
  std::map<std::string, float> select_values = {
      {POWER_STATUS, buffer_[D2_PACKET_POWER_IDX]},
      {OPERATIONAL_MODE, buffer_[D2_PACKET_MODE_IDX]},
      {P24_OFF_PEAK_MODE, buffer_[D2_PACKET_P24_IDX]},
      {P16_SOLAR_MODE_INT, buffer_[D2_PACKET_P16_IDX]},
      {P23_PV_MODE_INT, buffer_[D2_PACKET_P23_IDX]},
  };

  for (const auto &entry : select_values) {
    set_select_value(entry.first, entry.second);
  }

  print_buffer();
  return;
}

void DaikinEkhheComponent::parse_d4_packet() {
  print_buffer();
  return;
}

void DaikinEkhheComponent::parse_c1_packet() {
  print_buffer();
  return;
}

void DaikinEkhheComponent::parse_cc_packet() {
  print_buffer();
  return;
}

void DaikinEkhheComponent::register_sensor(const std::string &sensor_name, esphome::sensor::Sensor *sensor) {
  if (sensor != nullptr) {
    sensors_[sensor_name] = sensor;
    ESP_LOGI(TAG, "Registered Sensor: %s", sensor_name.c_str());
  }
}


void DaikinEkhheComponent::register_binary_sensor(const std::string &sensor_name, esphome::binary_sensor::BinarySensor *binary_sensor) {
  if (binary_sensor != nullptr) {
    binary_sensors_[sensor_name] = binary_sensor;
    ESP_LOGI(TAG, "Registered Binary Sensor: %s", sensor_name.c_str());
  }
}

void DaikinEkhheComponent::register_number(const std::string &number_name, esphome::number::Number *number) {
  if (number != nullptr) {
    numbers_[number_name] = number;
    ESP_LOGI(TAG, "Registered Number: %s", number_name.c_str());
  }
}

void DaikinEkhheComponent::register_select(const std::string &select_name, select::Select *select) {
  if (select != nullptr) {
      //selects_[select_name] = select;
      selects_[select_name] = static_cast<DaikinEkhheSelect *>(select);
      ESP_LOGI(TAG, "Registered Select: %s", select_name.c_str());
  }
}


void DaikinEkhheComponent::set_sensor_value(const std::string &sensor_name, float value) {
  if (sensors_.find(sensor_name) != sensors_.end()) {
    sensors_[sensor_name]->publish_state(value);
  }
}


void DaikinEkhheComponent::set_binary_sensor_value(const std::string &sensor_name, bool value) {
  if (binary_sensors_.find(sensor_name) != binary_sensors_.end()) {
    binary_sensors_[sensor_name]->publish_state(value);
  }
}


void DaikinEkhheComponent::set_number_value(const std::string &number_name, float value) {
  // This sets a number value that's been gotten from the UART stream, not something that's 
  // set through the API or UI
  if (numbers_.find(number_name) != numbers_.end()) {
    numbers_[number_name]->publish_state(value);
  }
}

void DaikinEkhheComponent::set_select_value(const std::string &select_name, int value) {
  // This sets a select value that's been gotten from the UART stream, not something that's 
  // set through the API or UI

  if (selects_.count(select_name)) {
    DaikinEkhheSelect *select = selects_[select_name];

        // Find the corresponding string option for the numeric value
        for (const auto &entry : select->get_select_mappings()) {
            if (entry.second == value) {                
                // Update ESPHome with the new selected value
                select->publish_state(entry.first);
                return;
            }
        }
  }
}

void DaikinEkhheComponent::send_uart_command(int number_id, float value) {
  ESP_LOGI(TAG, "Sending UART command: Number ID %d -> Value %.2f", number_id, value);
  
  // TODO: Format the UART command correctly according to the device protocol
  char command[20];
  snprintf(command, sizeof(command), "SET %d %.2f\n", number_id, value);
  
  // Send the command over UART
  this->write_str(command);
}

void DaikinEkhheComponent::send_uart_switch_command(int switch_id, bool state) {
  ESP_LOGI(TAG, "Sending UART switch command: Switch ID %d -> State %s", switch_id, state ? "ON" : "OFF");
  
  // TODO: Format the UART command correctly according to the device protocol
  char command[20];
  snprintf(command, sizeof(command), "SWITCH %d %d\n", switch_id, state ? 1 : 0);
  
  // Send the command over UART
  this->write_str(command);
}


uint8_t DaikinEkhheComponent::ekhhe_checksum(const std::vector<uint8_t>& data_bytes) {
  // Compute the checksum as (sum of data bytes) mod 256 + 170
  uint16_t sum = std::accumulate(data_bytes.begin(), data_bytes.end()-1, 0);
  return (sum % 256 + 170) & 0xFF;
}

void DaikinEkhheComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Daikin EKHHE:");

    // Log all enabled sensors
    ESP_LOGCONFIG(TAG, "Enabled Sensors:");
    for (const auto &entry : sensors_) {
        if (entry.second != nullptr) {
            ESP_LOGCONFIG(TAG, "  - %s: Last value: %.2f", entry.first.c_str(), entry.second->get_state());
        }
    }


    // needs to be 9600/N/1
    this->check_uart_settings(9600, 1, esphome::uart::UART_CONFIG_PARITY_NONE, 8);
}

void DaikinEkhheComponent::on_shutdown() {
    ESP_LOGI(TAG, "Shutdown was called");
}

void DaikinEkhheComponent::update() {
    ESP_LOGI(TAG, "Update was called");
}


void DaikinEkhheNumber::control(float value) {
    ESP_LOGI(TAG, "Number control called: %.2f", value);

    // Update value in ESPHome
    this->publish_state(value);

    // Send command via UART (example)
    // Will need to later implement this to control numbers over UART
    //send_uart_command(value);
}

void DaikinEkhheSelect::control(const std::string &value) {
    ESP_LOGI(TAG, "Select contro called: %s", value.c_str());

    // Update value in ESPHome
    this->publish_state(value);

    // Send selection over UART
    // send_uart_command(value);  // Uncomment if needed
}

//void DaikinEkhheSelect::setSelectMappings(std::vector<uint8_t> mappings)
//{
//    this->mappings = std::move(mappings);
//}

}  // namespace daikin_ekkhe
}  // namespace esphome
