#include "esp_cam_sensor.h"

namespace esphome {
namespace esp_cam_sensor_esphome {

static const char *const TAG = "esp_cam_sensor";

#ifdef USE_ESP32_VARIANT_ESP32P4

esp_cam_sensor_device_t* ESPCamSensorWrapper::detect_sensor(
  i2c_master_bus_handle_t i2c_handle,
  int8_t reset_pin,
  int8_t pwdn_pin,
  uint8_t sccb_addr
) {
  esp_sccb_io_handle_t sccb_handle;
  sccb_i2c_config_t sccb_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = sccb_addr,
    .scl_speed_hz = 400000
  };
  
  esp_err_t ret = sccb_new_i2c_io(i2c_handle, &sccb_config, &sccb_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create SCCB handle: %d", ret);
    return nullptr;
  }
  
  esp_cam_sensor_config_t sensor_config = {
    .sccb_handle = sccb_handle,
    .reset_pin = reset_pin,
    .pwdn_pin = pwdn_pin,
    .xclk_pin = -1,
    .xclk_freq_hz = 24000000,
    .sensor_port = ESP_CAM_SENSOR_MIPI_CSI
  };
  
  // Parcourir les fonctions de détection
  for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
       p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
    
    if (p->port == ESP_CAM_SENSOR_MIPI_CSI && p->sccb_addr == sccb_addr) {
      esp_cam_sensor_device_t *dev = p->detect(&sensor_config);
      if (dev != nullptr) {
        ESP_LOGI(TAG, "Detected sensor: %s (ID: 0x%04X)", 
                 dev->name, dev->id.pid);
        return dev;
      }
    }
  }
  
  ESP_LOGE(TAG, "No sensor detected at address 0x%02X", sccb_addr);
  return nullptr;
}

esp_err_t ESPCamSensorWrapper::set_format(
  esp_cam_sensor_device_t* dev,
  const esp_cam_sensor_format_t* format
) {
  return esp_cam_sensor_set_format(dev, format);
}

esp_err_t ESPCamSensorWrapper::get_format(
  esp_cam_sensor_device_t* dev,
  esp_cam_sensor_format_t* format
) {
  return esp_cam_sensor_get_format(dev, format);
}

esp_err_t ESPCamSensorWrapper::start_stream(esp_cam_sensor_device_t* dev) {
  int enable = 1;
  return esp_cam_sensor_ioctl(dev, ESP_CAM_SENSOR_IOC_S_STREAM, &enable);
}

esp_err_t ESPCamSensorWrapper::stop_stream(esp_cam_sensor_device_t* dev) {
  int enable = 0;
  return esp_cam_sensor_ioctl(dev, ESP_CAM_SENSOR_IOC_S_STREAM, &enable);
}

#endif

}  // namespace esp_cam_sensor_esphome
}  // namespace esphome
