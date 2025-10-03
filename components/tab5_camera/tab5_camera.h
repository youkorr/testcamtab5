#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

// Forward declarations pour éviter les includes C dans le header
#ifdef USE_ESP32_VARIANT_ESP32P4
extern "C" {
  struct esp_cam_sensor_device_t;
  typedef struct esp_cam_sensor_device_t esp_cam_sensor_device_t;
  
  struct esp_cam_ctlr_t;
  typedef struct esp_cam_ctlr_t* esp_cam_ctlr_handle_t;
  
  struct isp_processor_t;
  typedef struct isp_processor_t* isp_proc_handle_t;
}
#endif

namespace esphome {
namespace tab5_camera {

enum CameraResolution {
  RESOLUTION_1080P = 0,
  RESOLUTION_720P = 1,
  RESOLUTION_VGA = 2,
  RESOLUTION_QVGA = 3,
};

enum PixelFormat {
  PIXEL_FORMAT_RGB565 = 0,
  PIXEL_FORMAT_YUV422 = 1,
  PIXEL_FORMAT_RAW8 = 2,
  PIXEL_FORMAT_JPEG = 3,
};

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
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_external_clock_pin(GPIOPin *pin) { this->xclk_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->xclk_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_sensor_address(uint8_t address) { this->sensor_address_ = address; }
  void set_name(const std::string &name) { this->name_ = name; }
  void set_resolution(CameraResolution resolution) { this->resolution_ = resolution; }
  void set_pixel_format(PixelFormat format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_framerate(uint8_t fps) { this->framerate_ = fps; }

  bool capture_frame();
  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();
  bool is_streaming() const { return this->streaming_; }
  
  uint8_t* get_image_data() { return this->frame_buffer_.buffer; }
  size_t get_image_size() const { return this->frame_buffer_.length; }
  uint16_t get_image_width() const { return this->frame_buffer_.width; }
  uint16_t get_image_height() const { return this->frame_buffer_.height; }

 protected:
  GPIOPin *xclk_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  uint32_t xclk_frequency_{24000000};
  uint8_t sensor_address_{0x36};
  std::string name_{"Tab5 Camera"};
  
  CameraResolution resolution_{RESOLUTION_VGA};
  PixelFormat pixel_format_{PIXEL_FORMAT_RGB565};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{30};
  
  bool initialized_{false};
  bool streaming_{false};
  bool sensor_detected_{false};
  CameraFrameBuffer frame_buffer_{};
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  esp_cam_sensor_device_t *sensor_device_{nullptr};
  esp_cam_ctlr_handle_t csi_handle_{nullptr};
  isp_proc_handle_t isp_handle_{nullptr};
  
  bool init_espressif_sensor_();
  bool init_csi_isp_();
#endif
  
  CameraResolutionInfo get_resolution_info_();
  void init_test_pattern_();
  
  static constexpr uint16_t SC2356_CHIP_ID_VALUE = 0xEB52;
};

}  // namespace tab5_camera
}  // namespace esphome




