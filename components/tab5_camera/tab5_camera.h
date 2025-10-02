#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include <vector>

namespace esphome {
namespace tab5_camera {

// Résolutions supportées par SC202CS/SC2356
enum CameraResolution {
  RESOLUTION_1080P = 0,  // 1920x1080
  RESOLUTION_720P = 1,   // 1280x720
  RESOLUTION_VGA = 2,    // 640x480
  RESOLUTION_QVGA = 3,   // 320x240
};

// Formats pixel supportés
enum PixelFormat {
  PIXEL_FORMAT_RGB565 = 0,
  PIXEL_FORMAT_YUV422 = 1,
  PIXEL_FORMAT_RAW8 = 2,
  PIXEL_FORMAT_JPEG = 3,
};

// Structures pour la configuration
struct CameraResolutionInfo {
  uint16_t width;
  uint16_t height;
};

struct CameraFrameBuffer {
  uint8_t *buffer;
  size_t length;
  uint16_t width;
  uint16_t height;
  PixelFormat format;
};

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  Tab5Camera() = default;

  // Setup et loop
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration des pins
  void set_external_clock_pin(GPIOPin *pin) { this->xclk_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->xclk_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  
  // Configuration du capteur
  void set_sensor_address(uint8_t address) { this->sensor_address_ = address; }
  void set_name(const std::string &name) { this->name_ = name; }
  
  // Configuration de l'image
  void set_resolution(CameraResolution resolution) { this->resolution_ = resolution; }
  void set_pixel_format(PixelFormat format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_framerate(uint8_t fps) { this->framerate_ = fps; }

  // Méthodes publiques pour capturer des images
  bool capture_frame();
  CameraFrameBuffer *get_frame_buffer();
  void return_frame_buffer();
  
  // Nouvelles méthodes pour LVGL
  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();
  bool is_streaming() const { return this->streaming_; }
  
  // Accès aux données brutes pour LVGL
  uint8_t* get_image_data() { return this->frame_buffer_.buffer; }
  size_t get_image_size() const { return this->frame_buffer_.length; }
  uint16_t get_image_width() const { return this->frame_buffer_.width; }
  uint16_t get_image_height() const { return this->frame_buffer_.height; }

 protected:
  // Pins
  GPIOPin *xclk_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  uint32_t xclk_frequency_{24000000};
  
  // Configuration I2C du capteur
  uint8_t sensor_address_{0x36};
  std::string name_{"Tab5 Camera"};
  
  // Configuration caméra
  CameraResolution resolution_{RESOLUTION_VGA};
  PixelFormat pixel_format_{PIXEL_FORMAT_RGB565};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{30};
  
  // État interne
  bool initialized_{false};
  bool streaming_{false};
  bool csi_initialized_{false};
  CameraFrameBuffer frame_buffer_{};
  
  // Handles CSI (définis seulement si disponibles)
  #ifdef CONFIG_ESP_CAM_SENSOR_ENABLED
  void *cam_sensor_{nullptr};  // esp_cam_sensor_device_t*
  #endif
  
  // Méthodes privées d'initialisation
  bool init_camera_();
  bool init_sc202cs_sensor_();
  bool configure_sc202cs_();
  bool start_external_clock_();
  bool reset_sensor_();
  
  // Méthodes de communication I2C avec le capteur
  bool write_sensor_reg_(uint16_t reg, uint8_t value);
  bool read_sensor_reg_(uint16_t reg, uint8_t &value);
  bool write_sensor_regs_(const uint16_t regs[][2], size_t count);
  
  // Méthodes de gestion des données
  CameraResolutionInfo get_resolution_info_();
  bool allocate_frame_buffer_();
  void free_frame_buffer_();
  
  // Méthodes CSI
  bool init_csi_interface_();
  bool capture_csi_frame_();
  bool generate_test_pattern_();  // Fallback pattern de test
  
  // Registres SC202CS / SC2356
  static constexpr uint16_t SC202CS_CHIP_ID_REG = 0x3107;
  static constexpr uint16_t SC202CS_CHIP_ID_VALUE = 0xCB1C;  // SC202CS
  static constexpr uint16_t SC2356_CHIP_ID_VALUE = 0xEB52;   // SC2356
  static constexpr uint16_t SC202CS_RESET_REG = 0x0103;
  static constexpr uint16_t SC202CS_SLEEP_REG = 0x0100;
  
  // Tables de configuration pour différentes résolutions
  const uint16_t *get_resolution_config_table_();
  size_t get_resolution_config_table_size_();
};

}  // namespace tab5_camera
}  // namespace esphome




