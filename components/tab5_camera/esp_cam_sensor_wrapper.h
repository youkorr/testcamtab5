#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32_VARIANT_ESP32P4
extern "C" {
  #include "../esp_cam_sensor_esphome/esp_cam_sensor.h"
  #include "../esp_cam_sensor_esphome/esp_cam_sensor_detect.h"
  #include "../esp_cam_sensor_esphome/esp_sccb_i2c.h"
}
#endif

namespace esphome {
namespace esp_cam_sensor_esphome {

#ifdef USE_ESP32_VARIANT_ESP32P4
// Wrapper pour les fonctions C
class ESPCamSensorWrapper {
public:
  static esp_cam_sensor_device_t* detect_sensor(
    i2c_master_bus_handle_t i2c_handle,
    int8_t reset_pin,
    int8_t pwdn_pin,
    uint8_t sccb_addr
  );
  
  static esp_err_t set_format(
    esp_cam_sensor_device_t* dev,
    const esp_cam_sensor_format_t* format
  );
  
  static esp_err_t get_format(
    esp_cam_sensor_device_t* dev,
    esp_cam_sensor_format_t* format
  );
  
  static esp_err_t start_stream(esp_cam_sensor_device_t* dev);
  static esp_err_t stop_stream(esp_cam_sensor_device_t* dev);
};
#endif

}  // namespace esp_cam_sensor_esphome
}  // namespace esphome
