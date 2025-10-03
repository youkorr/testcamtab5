#include "lvgl_camera_display.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace lvgl_camera_display {

static const char *const TAG = "lvgl_camera_display";

void LVGLCameraDisplay::setup() {
  ESP_LOGCONFIG(TAG, "🎥 Configuration LVGL Camera Display...");
  
  if (this->camera_ == nullptr) {
    ESP_LOGE(TAG, "❌ Camera non configurée");
    this->mark_failed();
    return;
  }
  
  ESP_LOGI(TAG, "✅ LVGL Camera Display initialisé");
  ESP_LOGI(TAG, "   Update interval: %u ms (~%d FPS)", 
           this->update_interval_, 1000 / this->update_interval_);
}

void LVGLCameraDisplay::loop() {
  uint32_t now = millis();
  
  // Vérifier si c'est le moment de mettre à jour
  if (now - this->last_update_ < this->update_interval_) {
    return;
  }
  
  this->last_update_ = now;
  
  // Compteur de frames pour debug
  static uint32_t loop_count = 0;
  loop_count++;
  
  // Logger l'état toutes les 30 loops
  if (loop_count % 30 == 0) {
    ESP_LOGD(TAG, "📊 Loop #%u - streaming=%d, canvas=%p", 
             loop_count, this->camera_->is_streaming(), this->canvas_obj_);
  }
  
  // Si la caméra est en streaming, capturer ET mettre à jour le canvas
  if (this->camera_->is_streaming()) {
    // Capturer une nouvelle frame
    bool frame_captured = this->camera_->capture_frame();
    
    if (frame_captured) {
      this->update_canvas_();
      
      // Compteur de frames pour debug
      static uint32_t frame_count = 0;
      frame_count++;
      
      // Logger toutes les 30 frames
      if (frame_count % 30 == 0) {
        ESP_LOGI(TAG, "✓ %u frames affichées", frame_count);
      }
    } else {
      ESP_LOGV(TAG, "⚠️  Frame capture échouée");
    }
  } else {
    // Logger si pas en streaming
    if (loop_count % 100 == 0) {
      ESP_LOGW(TAG, "⏸️  Caméra pas en streaming");
    }
  }
}

void LVGLCameraDisplay::dump_config() {
  ESP_LOGCONFIG(TAG, "LVGL Camera Display:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->update_interval_);
  ESP_LOGCONFIG(TAG, "  FPS cible: ~%d", 1000 / this->update_interval_);
  ESP_LOGCONFIG(TAG, "  Canvas configuré: %s", this->canvas_obj_ ? "OUI" : "NON");
}

void LVGLCameraDisplay::update_canvas_() {
  if (this->camera_ == nullptr) {
    ESP_LOGV(TAG, "❌ Camera null");
    return;
  }
  
  if (this->canvas_obj_ == nullptr) {
    ESP_LOGW(TAG, "❌ Canvas null - pas encore configuré?");
    return;
  }
  
  // Obtenir les données de l'image
  uint8_t* img_data = this->camera_->get_image_data();
  uint16_t width = this->camera_->get_image_width();
  uint16_t height = this->camera_->get_image_height();
  
  if (img_data == nullptr) {
    ESP_LOGW(TAG, "❌ Image data null");
    return;
  }
  
  // Debug: vérifier les premières valeurs
  static bool first_update = true;
  if (first_update) {
    ESP_LOGI(TAG, "🖼️  Premier update canvas:");
    ESP_LOGI(TAG, "   Dimensions: %ux%u", width, height);
    ESP_LOGI(TAG, "   Buffer: %p", img_data);
    ESP_LOGI(TAG, "   Premiers pixels (RGB565): %02X%02X %02X%02X %02X%02X", 
             img_data[0], img_data[1], img_data[2], img_data[3], img_data[4], img_data[5]);
    first_update = false;
  }
  
  // Mettre à jour le buffer du canvas avec les données RGB565
  lv_canvas_set_buffer(this->canvas_obj_, img_data, width, height, LV_IMG_CF_TRUE_COLOR);
  
  // Invalider le canvas pour forcer le redessinage
  lv_obj_invalidate(this->canvas_obj_);
  
  // Logger seulement toutes les 30 frames
  static uint32_t update_count = 0;
  update_count++;
  if (update_count % 30 == 0) {
    ESP_LOGI(TAG, "✓ Canvas update #%u: %ux%u", update_count, width, height);
  }
}

void LVGLCameraDisplay::configure_canvas(lv_obj_t *canvas) { 
  this->canvas_obj_ = canvas;
  ESP_LOGI(TAG, "🎨 Canvas configuré: %p", canvas);
  
  if (canvas != nullptr) {
    // Vérifier les propriétés du canvas
    lv_coord_t w = lv_obj_get_width(canvas);
    lv_coord_t h = lv_obj_get_height(canvas);
    ESP_LOGI(TAG, "   Taille canvas: %dx%d", w, h);
  }
}

}  // namespace lvgl_camera_display
}  // namespace esphome
