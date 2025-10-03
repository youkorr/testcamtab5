#include "tab5_camera.h"
#include "esphome/core/log.h"

// Headers ESP-IDF pour CSI/ISP (disponibles dans ESP-IDF 5.4+)
#ifdef USE_ESP32_VARIANT_ESP32P4
  #include "driver/isp.h"
  #include "esp_cam_ctlr.h"
  #include "esp_cam_ctlr_csi.h"
  
  // Headers pour I2C
  #include "driver/i2c_master.h"
#endif

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Initialisation Tab5 Camera...");
  
  // Allouer buffer RGB565
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_.width = res.width;
  this->frame_buffer_.height = res.height;
  this->frame_buffer_.length = res.width * res.height * 2;
  
  this->frame_buffer_.buffer = (uint8_t*)heap_caps_malloc(
    this->frame_buffer_.length, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (!this->frame_buffer_.buffer) {
    ESP_LOGE(TAG, "Échec allocation buffer");
    this->mark_failed();
    return;
  }
  
  ESP_LOGI(TAG, "Buffer alloué: %ux%u = %u bytes @ %p", 
           res.width, res.height, this->frame_buffer_.length, this->frame_buffer_.buffer);
  
  // Pattern de test
  this->init_test_pattern_();
  
  // Vérifier SC202CS via I2C
  uint16_t chip_id = this->read_chip_id_();
  if (chip_id == SC2356_CHIP_ID_VALUE) {
    ESP_LOGI(TAG, "✅ SC202CS détecté: 0x%04X", chip_id);
    this->sensor_detected_ = true;
  } else {
    ESP_LOGW(TAG, "⚠️  Chip ID: 0x%04X (attendu: 0x%04X)", chip_id, SC2356_CHIP_ID_VALUE);
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Tab5 Camera prête (mode test pattern)");
}

uint16_t Tab5Camera::read_chip_id_() {
  uint8_t id_h = 0, id_l = 0;
  uint8_t reg_buf[2];
  
  // Chip ID @ 0x3107-0x3108
  reg_buf[0] = 0x31;
  reg_buf[1] = 0x07;
  
  if (this->write_read(reg_buf, 2, &id_h, 1) == i2c::ERROR_OK) {
    reg_buf[0] = 0x31;
    reg_buf[1] = 0x08;
    this->write_read(reg_buf, 2, &id_l, 1);
    
    return (id_h << 8) | id_l;
  }
  
  return 0x0000;
}

void Tab5Camera::init_test_pattern_() {
  CameraResolutionInfo res = this->get_resolution_info_();
  
  for (size_t y = 0; y < res.height; y++) {
    uint16_t color;
    if (y < res.height / 3) {
      color = 0xF800; // Rouge
    } else if (y < 2 * res.height / 3) {
      color = 0x07E0; // Vert
    } else {
      color = 0x001F; // Bleu
    }
    
    for (size_t x = 0; x < res.width; x++) {
      size_t i = y * res.width + x;
      this->frame_buffer_.buffer[i * 2] = color & 0xFF;
      this->frame_buffer_.buffer[i * 2 + 1] = (color >> 8) & 0xFF;
    }
  }
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) {
    return false;
  }
  
  // Mode test animé
  static uint32_t frame_num = 0;
  frame_num++;
  
  CameraResolutionInfo res = this->get_resolution_info_();
  uint8_t phase = (frame_num / 30) % 3;
  
  for (size_t y = 0; y < res.height; y++) {
    uint16_t color;
    size_t band = y / (res.height / 3);
    size_t color_idx = (band + phase) % 3;
    
    switch (color_idx) {
      case 0: color = 0xF800; break;
      case 1: color = 0x07E0; break;
      case 2: color = 0x001F; break;
      default: color = 0xFFFF; break;
    }
    
    for (size_t x = 0; x < res.width; x++) {
      size_t i = y * res.width + x;
      this->frame_buffer_.buffer[i * 2] = color & 0xFF;
      this->frame_buffer_.buffer[i * 2 + 1] = (color >> 8) & 0xFF;
    }
  }
  
  return true;
}

bool Tab5Camera::start_streaming() {
  ESP_LOGI(TAG, "▶️  Streaming started");
  this->streaming_ = true;
  return true;
}

bool Tab5Camera::stop_streaming() {
  ESP_LOGI(TAG, "⏹️  Streaming stopped");
  this->streaming_ = false;
  return true;
}

bool Tab5Camera::take_snapshot() {
  return this->capture_frame();
}

CameraResolutionInfo Tab5Camera::get_resolution_info_() {
  switch (this->resolution_) {
    case RESOLUTION_1080P: return {1920, 1080};
    case RESOLUTION_720P: return {1280, 720};
    case RESOLUTION_VGA: return {640, 480};
    case RESOLUTION_QVGA: return {320, 240};
    default: return {640, 480};
  }
}

void Tab5Camera::loop() {}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  SC202CS: %s", this->sensor_detected_ ? "Détecté" : "Non détecté");
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
    this->frame_buffer_.width, this->frame_buffer_.height);
  ESP_LOGCONFIG(TAG, "  Buffer: %u bytes", this->frame_buffer_.length);
  ESP_LOGCONFIG(TAG, "  I2C: 0x%02X", this->sensor_address_);
}

}  // namespace tab5_camera
}  // namespace esphome



