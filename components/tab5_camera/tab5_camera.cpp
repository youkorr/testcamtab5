#include "tab5_camera.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Initialisation Tab5 Camera...");
  
  // Allouer le buffer frame directement
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_.width = res.width;
  this->frame_buffer_.height = res.height;
  this->frame_buffer_.length = res.width * res.height * 2; // RGB565 = 2 bytes/pixel
  this->frame_buffer_.format = this->pixel_format_;
  
  // Allouer dans PSRAM
  this->frame_buffer_.buffer = (uint8_t*)heap_caps_malloc(
    this->frame_buffer_.length, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (!this->frame_buffer_.buffer) {
    ESP_LOGE(TAG, "❌ Échec allocation buffer (%d bytes)", this->frame_buffer_.length);
    this->mark_failed();
    return;
  }
  
  // Initialiser avec un pattern de test (damier gris)
  for (size_t i = 0; i < this->frame_buffer_.length / 2; i++) {
    uint16_t color = ((i / res.width + i % res.width) % 2) ? 0x8410 : 0x4208; // Damier gris
    this->frame_buffer_.buffer[i * 2] = color & 0xFF;
    this->frame_buffer_.buffer[i * 2 + 1] = (color >> 8) & 0xFF;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Tab5 Camera initialisée: %ux%u RGB565", res.width, res.height);
  ESP_LOGI(TAG, "   Buffer: %d bytes en PSRAM", this->frame_buffer_.length);
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) {
    return false;
  }
  
  // TODO: Implémenter la capture réelle depuis le SC202CS via CSI
  // Pour l'instant, on génère un pattern de test animé
  static uint32_t frame_num = 0;
  frame_num++;
  
  // Animer le damier (défilement)
  uint16_t offset = (frame_num / 10) % this->frame_buffer_.width;
  for (size_t i = 0; i < this->frame_buffer_.length / 2; i++) {
    size_t x = i % this->frame_buffer_.width;
    size_t y = i / this->frame_buffer_.width;
    uint16_t color = ((x + y + offset) % 2) ? 0x8410 : 0x4208;
    this->frame_buffer_.buffer[i * 2] = color & 0xFF;
    this->frame_buffer_.buffer[i * 2 + 1] = (color >> 8) & 0xFF;
  }
  
  return true;
}

bool Tab5Camera::take_snapshot() {
  return this->capture_frame();
}

bool Tab5Camera::start_streaming() {
  ESP_LOGI(TAG, "📹 Démarrage streaming");
  this->streaming_ = true;
  return true;
}

bool Tab5Camera::stop_streaming() {
  ESP_LOGI(TAG, "⏹️  Arrêt streaming");
  this->streaming_ = false;
  return true;
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

void Tab5Camera::loop() {
  // Rien à faire en loop, la capture est déclenchée par lvgl_camera_display
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Nom: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
    this->frame_buffer_.width, this->frame_buffer_.height);
  ESP_LOGCONFIG(TAG, "  Format: RGB565");
  ESP_LOGCONFIG(TAG, "  Buffer: %d bytes", this->frame_buffer_.length);
  ESP_LOGCONFIG(TAG, "  Adresse I2C: 0x%02X", this->sensor_address_);
}

}  // namespace tab5_camera
}  // namespace esphome




