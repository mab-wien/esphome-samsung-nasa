#include "nasa_text_sensor.h"
#include <cstdio>

namespace esphome {
namespace samsung_nasa {

void NASA_TextSensor::on_receive(long value) {
  // Only process and publish if the numeric value has changed
  if (value != this->last_numeric_value_) {
    if (this->lookup_func_) {
      this->publish_state(this->lookup_func_(value));
    } else {
      this->publish_state(std::to_string(value));
    }
    this->last_numeric_value_ = value;
  }
}

void NASA_TextSensor::on_receive_structure(const uint8_t *data, uint8_t size) {
  // Text, wenn alle Bytes (ohne abschließende 0x00/0xFF/Leerzeichen) druckbares ASCII sind, sonst Hex „08 08 00 12 33“
  uint8_t n = size;
  while (n > 0 && (data[n - 1] == 0x00 || data[n - 1] == 0xFF || data[n - 1] == ' '))
    n--;
  bool text = n >= 2;
  for (uint8_t i = 0; i < n && text; i++)
    text = data[i] >= 0x20 && data[i] < 0x7F;
  std::string s;
  if (text) {
    s.assign(reinterpret_cast<const char *>(data), n);
  } else {
    char hex[4];
    for (uint8_t i = 0; i < size; i++) {
      snprintf(hex, sizeof(hex), i ? " %02X" : "%02X", data[i]);
      s += hex;
    }
  }
  if (s != this->last_structure_) {
    this->last_structure_ = s;
    this->publish_state(s);
  }
}

void NASA_TextSensor::set_parent(NASA_Controller *controller) { this->controller_ = controller; }

}  // namespace samsung_nasa
}  // namespace esphome
