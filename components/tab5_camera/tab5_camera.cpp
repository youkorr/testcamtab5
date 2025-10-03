#include "tab5_camera.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "🎥 Initialisation Tab5 Camera (mode test)...");
  
  // Allouer le buffer frame RGB565 dans PSRAM
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_.width = res.width;
  this->frame_buffer_.height = res.height;
  this->frame_buffer_.length = res.width * res.height * 2; // RGB565
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "📐 Résolution: %ux%u", res.width, res.height);
  ESP_LOGI(TAG, "💾 Buffer: %u bytes", this->frame_buffer_.length);
  
  this->frame_buffer_.buffer = (uint8_t*)heap_caps_malloc(
    this->frame_buffer_.length, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (!this->frame_buffer_.buffer) {
    ESP_LOGE(TAG, "❌ Échec allocation buffer");
    this->mark_failed();
    return;
  }
  
  ESP_LOGI(TAG, "✅ Buffer alloué: %p", this->frame_buffer_.buffer);
  
  // Pattern de test
  this->init_test_pattern_();
  
  // TODO: Détecter SC202CS via I2C
  ESP_LOGI(TAG, "⚠️  Mode test pattern (CSI/ISP pas encore implémenté)");
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Tab5 Camera prête");
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

bool Tab5Camera::detect_sc202cs_() {
  // TODO: Implémenter détection I2C
  return false;
}

esp_err_t Tab5Camera::read_register16_(uint16_t reg, uint8_t *value) {
  uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  return this->write(reg_buf, 2, false) || this->read(value, 1);
}

esp_err_t Tab5Camera::write_register16_(uint16_t reg, uint8_t value) {
  uint8_t data[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value};
  return this->write(data, 3);
}

bool Tab5Camera::init_csi_isp_pipeline_() {
  ESP_LOGW(TAG, "⚠️  CSI/ISP non disponible - utiliser mode test");
  return false;
}

void Tab5Camera::configure_sc202cs_() {
  // Vide pour l'instant
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) {
    return false;
  }
  
  static uint32_t frame_num = 0;
  frame_num++;
  
  // Mode test: animer le pattern
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
  ESP_LOGI(TAG, "▶️  Démarrage streaming");
  this->streaming_ = true;
  return true;
}

bool Tab5Camera::stop_streaming() {
  ESP_LOGI(TAG, "⏹️  Arrêt streaming");
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
  ESP_LOGCONFIG(TAG, "  Mode: TEST PATTERN");
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
    this->frame_buffer_.width, this->frame_buffer_.height);
  ESP_LOGCONFIG(TAG, "  Buffer: %d bytes @ %p", 
    this->frame_buffer_.length, this->frame_buffer_.buffer);
}

}  // namespace tab5_camera
}  // namespace esphome





