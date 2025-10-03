#include "tab5_camera.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32_VARIANT_ESP32P4
  #include "esp_cam_ctlr.h"
  #include "esp_cam_ctlr_csi.h"
#endif

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Init Tab5 Camera...");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_.width = res.width;
  this->frame_buffer_.height = res.height;
  this->frame_buffer_.length = res.width * res.height * 2;
  
  this->frame_buffer_.buffer = (uint8_t*)heap_caps_malloc(
    this->frame_buffer_.length, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (!this->frame_buffer_.buffer) {
    ESP_LOGE(TAG, "Buffer allocation failed");
    this->mark_failed();
    return;
  }
  
  ESP_LOGI(TAG, "Buffer: %ux%u @ %p", res.width, res.height, this->frame_buffer_.buffer);
  
  this->init_test_pattern_();
  
  uint16_t chip_id = this->read_chip_id_();
  if (chip_id == SC2356_CHIP_ID_VALUE) {
    ESP_LOGI(TAG, "SC202CS detected: 0x%04X", chip_id);
    this->sensor_detected_ = true;
  } else {
    ESP_LOGW(TAG, "Chip ID: 0x%04X", chip_id);
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "Camera ready (test mode)");
}

uint16_t Tab5Camera::read_chip_id_() {
  uint8_t id_h = 0, id_l = 0;
  uint8_t reg[2] = {0x31, 0x07};
  
  if (this->write_read(reg, 2, &id_h, 1) == i2c::ERROR_OK) {
    reg[1] = 0x08;
    this->write_read(reg, 2, &id_l, 1);
    return (id_h << 8) | id_l;
  }
  return 0x0000;
}

void Tab5Camera::init_test_pattern_() {
  CameraResolutionInfo res = this->get_resolution_info_();
  
  for (size_t y = 0; y < res.height; y++) {
    uint16_t color = (y < res.height/3) ? 0xF800 : (y < 2*res.height/3) ? 0x07E0 : 0x001F;
    for (size_t x = 0; x < res.width; x++) {
      size_t i = y * res.width + x;
      this->frame_buffer_.buffer[i*2] = color & 0xFF;
      this->frame_buffer_.buffer[i*2+1] = (color >> 8) & 0xFF;
    }
  }
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) return false;
  
  static uint32_t frame = 0;
  frame++;
  
  CameraResolutionInfo res = this->get_resolution_info_();
  uint8_t phase = (frame / 30) % 3;
  
  for (size_t y = 0; y < res.height; y++) {
    size_t color_idx = (y / (res.height/3) + phase) % 3;
    uint16_t color = (color_idx == 0) ? 0xF800 : (color_idx == 1) ? 0x07E0 : 0x001F;
    
    for (size_t x = 0; x < res.width; x++) {
      size_t i = y * res.width + x;
      this->frame_buffer_.buffer[i*2] = color & 0xFF;
      this->frame_buffer_.buffer[i*2+1] = (color >> 8) & 0xFF;
    }
  }
  
  return true;
}

bool Tab5Camera::start_streaming() {
  ESP_LOGI(TAG, "Streaming ON");
  this->streaming_ = true;
  return true;
}

bool Tab5Camera::stop_streaming() {
  ESP_LOGI(TAG, "Streaming OFF");
  this->streaming_ = false;
  return true;
}

bool Tab5Camera::take_snapshot() {
  return this->capture_frame();
}

CameraResolutionInfo Tab5Camera::get_resolution_info_() {
  switch (this->resolution_) {
    case RESOLUTION_720P: return {1280, 720};
    case RESOLUTION_VGA: return {640, 480};
    case RESOLUTION_QVGA: return {320, 240};
    default: return {640, 480};
  }
}

void Tab5Camera::loop() {}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  SC202CS: %s", this->sensor_detected_ ? "Detected" : "Test");
  ESP_LOGCONFIG(TAG, "  Resolution: %ux%u", this->frame_buffer_.width, this->frame_buffer_.height);
}

}  // namespace tab5_camera
}  // namespace esphome



