#include "tab5_camera.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "🎥 Initialisation Tab5 Camera (ESP32-P4)...");
  
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
  
  // Pattern de test initial
  this->init_test_pattern_();
  
  // DIAGNOSTIC I2C
  ESP_LOGI(TAG, "🔍 === DIAGNOSTIC I2C SC202CS ===");
  ESP_LOGI(TAG, "Adresse I2C configurée: 0x%02X", this->sensor_address_);
  
  // Tester plusieurs adresses possibles
  uint8_t test_addresses[] = {0x36, 0x10, 0x30, 0x3C};
  
  for (uint8_t addr : test_addresses) {
    ESP_LOGI(TAG, "Test adresse 0x%02X...", addr);
    
    // Scan I2C simple
    if (this->parent_->write(addr, nullptr, 0) == i2c::ERROR_OK) {
      ESP_LOGI(TAG, "  ✓ Device répond à 0x%02X", addr);
      
      // Essayer de lire le chip ID
      uint16_t chip_id = this->read_chip_id_(addr);
      ESP_LOGI(TAG, "  Chip ID @ 0x%02X: 0x%04X", addr, chip_id);
      
      if (chip_id == SC2356_CHIP_ID_VALUE || chip_id == 0xCB5C || chip_id == 0x2356) {
        ESP_LOGI(TAG, "  ✅ SC202CS trouvé à 0x%02X !", addr);
        this->sensor_address_ = addr;
        this->sensor_detected_ = true;
        break;
      }
    }
  }
  
  if (!this->sensor_detected_) {
    ESP_LOGW(TAG, "⚠️  SC202CS non détecté - mode test pattern");
    ESP_LOGI(TAG, "Vérifiez:");
    ESP_LOGI(TAG, "  1. GPIO expander pour power/reset caméra");
    ESP_LOGI(TAG, "  2. Bus I2C correct (port/pins)");
    ESP_LOGI(TAG, "  3. Câble/connexion caméra");
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Tab5 Camera prête (mode: %s)", 
           this->sensor_detected_ ? "DÉTECTÉ" : "TEST");
}

uint16_t Tab5Camera::read_chip_id_(uint8_t addr) {
  uint8_t id_h = 0, id_l = 0;
  
  // Essayer plusieurs registres de chip ID possibles
  uint16_t id_regs[] = {0x3107, 0x0000, 0x0001, 0x300A};
  
  for (uint16_t reg : id_regs) {
    uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    
    if (this->parent_->write(addr, reg_buf, 2, false) == i2c::ERROR_OK) {
      if (this->parent_->read(addr, &id_h, 1) == i2c::ERROR_OK) {
        this->parent_->write(addr, reg_buf, 2, false);
        this->parent_->read(addr, &id_l, 1);
        
        uint16_t chip_id = (id_h << 8) | id_l;
        if (chip_id != 0x0000 && chip_id != 0xFFFF) {
          ESP_LOGD(TAG, "  Reg 0x%04X = 0x%04X", reg, chip_id);
          return chip_id;
        }
      }
    }
  }
  
  return 0x0000;
}

esp_err_t Tab5Camera::read_register16_(uint16_t reg, uint8_t *value) {
  uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  if (this->write(reg_buf, 2, false) != ESP_OK) {
    return ESP_FAIL;
  }
  return this->read(value, 1);
}

esp_err_t Tab5Camera::write_register16_(uint16_t reg, uint8_t value) {
  uint8_t data[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value};
  return this->write(data, 3);
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
  ESP_LOGCONFIG(TAG, "  Capteur: %s", this->sensor_detected_ ? "SC202CS" : "Non détecté");
  ESP_LOGCONFIG(TAG, "  Adresse I2C: 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
    this->frame_buffer_.width, this->frame_buffer_.height);
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->sensor_detected_ ? "READY" : "TEST PATTERN");
}

}  // namespace tab5_camera
}  // namespace esphome





