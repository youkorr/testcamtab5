#include "tab5_camera.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "🎥 Initialisation Tab5 Camera...");
  
  // Allouer le buffer frame directement
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_.width = res.width;
  this->frame_buffer_.height = res.height;
  this->frame_buffer_.length = res.width * res.height * 2; // RGB565 = 2 bytes/pixel
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "📐 Résolution: %ux%u", res.width, res.height);
  ESP_LOGI(TAG, "💾 Buffer size: %u bytes", this->frame_buffer_.length);
  
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
  
  ESP_LOGI(TAG, "✅ Buffer alloué à: %p", this->frame_buffer_.buffer);
  
  // Initialiser avec un pattern de test VISIBLE (bandes de couleur RGB)
  // Rouge en haut, Vert au milieu, Bleu en bas
  for (size_t y = 0; y < res.height; y++) {
    uint16_t color;
    if (y < res.height / 3) {
      color = 0xF800; // Rouge: RGB565 = 11111 000000 00000
    } else if (y < 2 * res.height / 3) {
      color = 0x07E0; // Vert: RGB565 = 00000 111111 00000
    } else {
      color = 0x001F; // Bleu: RGB565 = 00000 000000 11111
    }
    
    for (size_t x = 0; x < res.width; x++) {
      size_t i = y * res.width + x;
      this->frame_buffer_.buffer[i * 2] = color & 0xFF;         // LSB first
      this->frame_buffer_.buffer[i * 2 + 1] = (color >> 8) & 0xFF; // MSB
    }
  }
  
  ESP_LOGI(TAG, "🎨 Pattern de test initialisé (bandes RGB)");
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Tab5 Camera prête");
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) {
    ESP_LOGW(TAG, "⚠️  capture_frame() appelé mais pas initialisé");
    return false;
  }
  
  // Pattern de test animé: rotation des couleurs
  static uint32_t frame_num = 0;
  frame_num++;
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  // Faire tourner les bandes de couleur toutes les 30 frames
  uint8_t phase = (frame_num / 30) % 3;
  
  for (size_t y = 0; y < res.height; y++) {
    uint16_t color;
    size_t band = y / (res.height / 3);
    size_t color_idx = (band + phase) % 3;
    
    switch (color_idx) {
      case 0: color = 0xF800; break; // Rouge
      case 1: color = 0x07E0; break; // Vert
      case 2: color = 0x001F; break; // Bleu
      default: color = 0xFFFF; break; // Blanc
    }
    
    for (size_t x = 0; x < res.width; x++) {
      size_t i = y * res.width + x;
      this->frame_buffer_.buffer[i * 2] = color & 0xFF;
      this->frame_buffer_.buffer[i * 2 + 1] = (color >> 8) & 0xFF;
    }
  }
  
  // Logger périodiquement
  if (frame_num % 60 == 1) {
    ESP_LOGD(TAG, "🎬 Frame #%u capturée (phase=%u)", frame_num, phase);
  }
  
  return true;
}

bool Tab5Camera::take_snapshot() {
  return this->capture_frame();
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
  // Rien en loop
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Nom: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
    this->frame_buffer_.width, this->frame_buffer_.height);
  ESP_LOGCONFIG(TAG, "  Format: RGB565");
  ESP_LOGCONFIG(TAG, "  Buffer: %d bytes @ %p", 
    this->frame_buffer_.length, this->frame_buffer_.buffer);
  ESP_LOGCONFIG(TAG, "  Adresse I2C: 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_ ? "OUI" : "NON");
}

}  // namespace tab5_camera
}  // namespace esphome




