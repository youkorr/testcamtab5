#include "lvgl_camera_display.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace lvgl_camera_display {

static const char *const TAG = "lvgl_camera_display";

void LVGLCameraDisplay::setup() {
  ESP_LOGCONFIG(TAG, "Configuration LVGL Camera Display...");
  
  if (this->camera_ == nullptr) {
    ESP_LOGE(TAG, "Camera non configurée");
    this->mark_failed();
    return;
  }
  
  // Le canvas sera configuré plus tard via configure_canvas()
  // ou directement dans le loop quand LVGL est prêt
  ESP_LOGI(TAG, "LVGL Camera Display initialisé (canvas ID: %s)", this->canvas_id_.c_str());
}

void LVGLCameraDisplay::loop() {
  uint32_t now = millis();
  
  // Vérifier si c'est le moment de mettre à jour
  if (now - this->last_update_ < this->update_interval_) {
    return;
  }
  
  this->last_update_ = now;
  
  // Si la caméra est en streaming, mettre à jour le canvas
  if (this->camera_->is_streaming()) {
    this->update_canvas_();
  }
}

void LVGLCameraDisplay::dump_config() {
  ESP_LOGCONFIG(TAG, "LVGL Camera Display:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->update_interval_);
  ESP_LOGCONFIG(TAG, "  FPS: ~%d", 1000 / this->update_interval_);
}

void LVGLCameraDisplay::update_canvas_() {
  if (this->camera_ == nullptr || this->canvas_obj_ == nullptr) {
    return;
  }
  
  // Obtenir les données de l'image
  uint8_t* img_data = this->camera_->get_image_data();
  uint16_t width = this->camera_->get_image_width();
  uint16_t height = this->camera_->get_image_height();
  
  if (img_data == nullptr) {
    return;
  }
  
  // Mettre à jour le buffer du canvas avec les données RGB565
  lv_canvas_set_buffer(this->canvas_obj_, img_data, width, height, LV_IMG_CF_TRUE_COLOR);
  
  // Invalider le canvas pour forcer le redessinage
  lv_obj_invalidate(this->canvas_obj_);
  
  ESP_LOGV(TAG, "Canvas mis à jour: %ux%u", width, height);
}

}  // namespace lvgl_camera_display
}  // namespace esphome
