#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32_VARIANT_ESP32P4
extern "C" {
  #include "esp_video.h"
  //#include "esp_video_init.h"
  #include "esp_video_device_internal.h"
  #include "esp_cam_sensor.h"
  #include "esp_cam_ctlr_csi.h"
  #include "driver/isp.h"
  #include "esp_ldo_regulator.h"
}
#endif

namespace esphome {
namespace esp_video_esphome {

#ifdef USE_ESP32_VARIANT_ESP32P4

struct VideoConfig {
  uint16_t width;
  uint16_t height;
  uint32_t pixel_format;  // V4L2 format
  uint8_t fps;
};

class ESPVideoWrapper {
 public:
  // Initialisation du système vidéo
  static esp_err_t init_video_system(esp_cam_sensor_device_t *sensor_device);
  
  // Gestion du flux vidéo
  static esp_err_t start_capture();
  static esp_err_t stop_capture();
  
  // Configuration
  static esp_err_t set_format(const VideoConfig &config);
  static esp_err_t get_format(VideoConfig &config);
  
  // Récupération de frame
  static uint8_t* get_frame_buffer(size_t *size);
  static void release_frame_buffer(uint8_t *buffer);
  
  // État
  static bool is_streaming();
  
  // Nettoyage
  static void cleanup();
  
 private:
  static struct esp_video *video_device_;
  static bool initialized_;
  static bool streaming_;
  static esp_ldo_channel_handle_t ldo_handle_;
};

#endif

}  // namespace esp_video_esphome
}  // namespace esphome
