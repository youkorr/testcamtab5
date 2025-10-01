#include "tab5_camera_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera.sensor";

void Tab5CameraSensor::setup() {
  ESP_LOGCONFIG(TAG, "Configuration Tab5 Camera Sensor...");
  
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent Tab5Camera non configuré");
    this->mark_failed();
    return;
  }
}

void Tab5CameraSensor::update() {
  this->capture_and_publish_();
}

void Tab5CameraSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera Sensor:");
  LOG_UPDATE_INTERVAL(this);
}

void Tab5CameraSensor::capture_and_publish_() {
  if (this->parent_ == nullptr) {
    return;
  }
  
  // Capturer une image
  if (!this->parent_->capture_frame()) {
    ESP_LOGW(TAG, "Échec de la capture d'image");
    return;
  }
  
  // Récupérer le buffer
  CameraFrameBuffer *fb = this->parent_->get_frame_buffer();
  if (fb == nullptr || fb->buffer == nullptr) {
    ESP_LOGE(TAG, "Buffer invalide");
    return;
  }
  
  // Publier l'image via l'interface caméra ESPHome
  this->publish_image(fb->buffer, fb->length);
  
  // Retourner le buffer
  this->parent_->return_frame_buffer();
}

}  // namespace tab5_camera
}  // namespace esphome
