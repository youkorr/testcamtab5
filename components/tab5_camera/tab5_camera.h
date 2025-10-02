#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esp_sccb_intf.h"

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
  CameraFrameBuffer frame_buffer_{};
  
  esp_sccb_io_handle_t sccb_handle_{nullptr};
  
  bool init_sensor_with_official_driver_();
  bool init_sc202cs_manually_();
  bool allocate_frame_buffer_();
  void free_frame_buffer_();
  bool start_external_clock_();
  void get_resolution_dimensions_(uint16_t &width, uint16_t &height);
};

}  // namespace tab5_camera
}  // namespace esphome




