#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

// Includes ESP-IDF pour V4L2
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

// V4L2 et esp_video pour ESP32-P4
#ifdef USE_ESP32_VARIANT_ESP32P4
  // Utiliser notre header V4L2 minimal local
  #include "v4l2_minimal.h"
  #include "esp_video_init.h"
  
  // Définir mmap manuellement si non disponible
  #ifndef MAP_FAILED
    #define MAP_FAILED ((void *) -1)
    #define PROT_READ  0x1
    #define PROT_WRITE 0x2
    #define MAP_SHARED 0x01
  #endif
  
  // Déclarer mmap si pas dans les headers standards
  extern "C" {
    void* mmap(void* addr, size_t length, int prot, int flags, int fd, off_t offset);
    int munmap(void* addr, size_t length);
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
  CameraFrameBuffer frame_buffer_{};
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  // V4L2 handles
  int video_fd_{-1};
  uint8_t *mmap_buffers_[2]{nullptr, nullptr};
  
  bool init_esp_video_();
  bool open_v4l2_device_();
  bool init_v4l2_buffers_();
  bool start_v4l2_stream_();
  bool capture_v4l2_frame_();
#endif
  
  CameraResolutionInfo get_resolution_info_();
  
  static constexpr uint16_t SC202CS_CHIP_ID_REG = 0x3107;
  static constexpr uint16_t SC2356_CHIP_ID_VALUE = 0xEB52;
};

}  // namespace tab5_camera
}  // namespace esphome




