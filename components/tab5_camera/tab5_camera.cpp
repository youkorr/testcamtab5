#include "tab5_camera.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "🎥 Initialisation Tab5 Camera SC202CS (RAW→RGB565)...");
  
  // Allouer le buffer frame RGB565 dans PSRAM
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_.width = res.width;
  this->frame_buffer_.height = res.height;
  this->frame_buffer_.length = res.width * res.height * 2; // RGB565 = 2 bytes/pixel
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "📐 Résolution: %ux%u", res.width, res.height);
  ESP_LOGI(TAG, "💾 Buffer RGB565: %u bytes", this->frame_buffer_.length);
  
  this->frame_buffer_.buffer = (uint8_t*)heap_caps_malloc(
    this->frame_buffer_.length, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (!this->frame_buffer_.buffer) {
    ESP_LOGE(TAG, "❌ Échec allocation buffer RGB565");
    this->mark_failed();
    return;
  }
  
  ESP_LOGI(TAG, "✅ Buffer RGB565 alloué: %p", this->frame_buffer_.buffer);
  
  // Initialiser le pattern de test
  this->init_test_pattern_();
  
  // Tenter de détecter le SC202CS
  ESP_LOGI(TAG, "🔍 Détection SC202CS @ I2C 0x%02X...", this->sensor_address_);
  
  if (this->detect_sc202cs_()) {
    ESP_LOGI(TAG, "✅ SC202CS détecté !");
    
    // Initialiser CSI + ISP pour RAW8 → RGB565
    if (this->init_csi_isp_pipeline_()) {
      ESP_LOGI(TAG, "✅ Pipeline CSI→ISP→RGB565 initialisé");
      this->sensor_detected_ = true;
    } else {
      ESP_LOGW(TAG, "⚠️  Échec init pipeline - mode test");
    }
  } else {
    ESP_LOGW(TAG, "⚠️  SC202CS non détecté - mode test pattern");
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Tab5 Camera prête");
}

bool Tab5Camera::detect_sc202cs_() {
  // Lire le Chip ID (registre 0x3107)
  uint8_t id_h = 0, id_l = 0;
  
  // SC202CS utilise des registres 16-bit
  if (this->read_register16_(0x3107, &id_h) != ESP_OK) {
    ESP_LOGV(TAG, "Échec lecture chip ID");
    return false;
  }
  
  if (this->read_register16_(0x3108, &id_l) != ESP_OK) {
    return false;
  }
  
  uint16_t chip_id = (id_h << 8) | id_l;
  ESP_LOGI(TAG, "📟 Chip ID: 0x%04X (attendu: 0x%04X)", chip_id, SC2356_CHIP_ID_VALUE);
  
  return (chip_id == SC2356_CHIP_ID_VALUE);
}

esp_err_t Tab5Camera::read_register16_(uint16_t reg, uint8_t *value) {
  // SC202CS utilise des adresses 16-bit
  uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  return this->write(reg_buf, 2, false) || this->read(value, 1);
}

esp_err_t Tab5Camera::write_register16_(uint16_t reg, uint8_t value) {
  uint8_t data[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value};
  return this->write(data, 3);
}

bool Tab5Camera::init_csi_isp_pipeline_() {
  ESP_LOGI(TAG, "🎬 Initialisation pipeline CSI→ISP...");
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  CameraResolutionInfo res = this->get_resolution_info_();
  
  // ========== 1. CONFIGURATION CSI (entrée RAW8) ==========
  esp_cam_ctlr_csi_config_t csi_config = {
    .ctlr_id = 0,
    .h_res = res.width,
    .v_res = res.height,
    .lane_bit_rate_mbps = 800,  // 800 Mbps par lane
    .input_data_color_type = CAM_CTLR_COLOR_RAW8,     // SC202CS → RAW8
    .output_data_color_type = CAM_CTLR_COLOR_RAW8,    // CSI → ISP en RAW8
    .data_lane_num = 2,  // 2 lanes MIPI
    .byte_swap_en = false,
    .queue_items = 2,
  };
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "❌ esp_cam_new_csi_ctlr: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "✓ CSI configuré: RAW8, 2 lanes @ 800Mbps");
  
  // ========== 2. CONFIGURATION ISP (RAW8 → RGB565) ==========
  isp_processor_cfg_t isp_config = {
    .clk_hz = 80000000,  // 80 MHz pour l'ISP
    .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
    .input_data_color_format = ISP_COLOR_RAW8,
    .output_data_color_format = ISP_COLOR_RGB565,
    .has_line_start_packet = false,
    .has_line_end_packet = false,
    .h_res = res.width,
    .v_res = res.height,
    .bayer_type = ISP_BAYER_BGGR,  // SC202CS utilise BGGR Bayer pattern
  };
  
  ret = isp_new_processor(&isp_config, &this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "❌ isp_new_processor: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "✓ ISP configuré: RAW8→RGB565, Bayer BGGR");
  
  // ========== 3. CALLBACK pour récupérer les frames RGB565 ==========
  isp_evt_cbs_t isp_cbs = {
    .on_frame_done = on_isp_frame_callback_,
  };
  
  ret = isp_register_event_callbacks(this->isp_handle_, &isp_cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "❌ ISP callbacks: %s", esp_err_to_name(ret));
    return false;
  }
  
  // ========== 4. ACTIVER LE PIPELINE ==========
  ret = isp_enable(this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "❌ isp_enable: %s", esp_err_to_name(ret));
    return false;
  }
  
  ret = esp_cam_ctlr_enable(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "❌ CSI enable: %s", esp_err_to_name(ret));
    return false;
  }
  
  // ========== 5. CONFIGURER LE SC202CS ==========
  this->configure_sc202cs_();
  
  ESP_LOGI(TAG, "✅ Pipeline CSI→ISP→RGB565 actif");
  return true;
#else
  ESP_LOGW(TAG, "CSI/ISP non supporté sur cette plateforme");
  return false;
#endif
}

void Tab5Camera::configure_sc202cs_() {
  ESP_LOGI(TAG, "⚙️  Configuration SC202CS pour RAW8...");
  
  // Reset software
  this->write_register16_(0x0103, 0x01);
  delay(10);
  
  // Configuration pour VGA (640x480) RAW8
  // TODO: Charger les registres complets du fabricant
  // Voici une configuration minimale:
  
  // PLL settings pour clock
  this->write_register16_(0x0303, 0x01);  // PLL multiplier
  this->write_register16_(0x0307, 0x32);  // PLL multiplier value
  
  // Frame size: VGA
  this->write_register16_(0x0340, 0x02);  // Frame length high
  this->write_register16_(0x0341, 0x58);  // Frame length low (600)
  this->write_register16_(0x0342, 0x03);  // Line length high  
  this->write_register16_(0x0343, 0x20);  // Line length low (800)
  
  // Output size: 640x480
  this->write_register16_(0x034C, 0x02);  // Width high
  this->write_register16_(0x034D, 0x80);  // Width low (640)
  this->write_register16_(0x034E, 0x01);  // Height high
  this->write_register16_(0x034F, 0xE0);  // Height low (480)
  
  // Format: RAW8
  this->write_register16_(0x0112, 0x08);  // RAW8
  this->write_register16_(0x0113, 0x08);  // RAW8
  
  // Start streaming
  this->write_register16_(0x0100, 0x01);
  
  ESP_LOGI(TAG, "✓ SC202CS configuré: 640x480 RAW8 BGGR");
}

bool Tab5Camera::on_isp_frame_callback_(isp_proc_handle_t proc, 
                                        isp_trans_t *trans, 
                                        void *user_data) {
  Tab5Camera *camera = (Tab5Camera*)user_data;
  
  if (trans && trans->buffer && trans->buflen > 0) {
    // Copier la frame RGB565 depuis l'ISP
    size_t copy_len = std::min((size_t)trans->buflen, camera->frame_buffer_.length);
    memcpy(camera->frame_buffer_.buffer, trans->buffer, copy_len);
    
    camera->frame_received_ = true;
    
    static uint32_t isp_frames = 0;
    isp_frames++;
    if (isp_frames % 60 == 0) {
      ESP_LOGI(TAG, "📸 ISP RGB565 frame #%u (%u bytes)", isp_frames, trans->buflen);
    }
  }
  
  return false;
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
  
  if (this->sensor_detected_ && this->isp_handle_) {
    // Mode réel: l'ISP envoie automatiquement les frames via callback
    // On vérifie juste qu'on reçoit des données
    
    if (this->frame_received_) {
      this->frame_received_ = false;  // Reset pour la prochaine
      return true;
    } else {
      ESP_LOGV(TAG, "En attente frame ISP...");
      return false;
    }
  } else {
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
}

bool Tab5Camera::start_streaming() {
  ESP_LOGI(TAG, "▶️  Démarrage streaming");
  this->streaming_ = true;
  
  if (this->sensor_detected_ && this->csi_handle_) {
    esp_cam_ctlr_start(this->csi_handle_);
    isp_start(this->isp_handle_);
  }
  
  return true;
}

bool Tab5Camera::stop_streaming() {
  ESP_LOGI(TAG, "⏹️  Arrêt streaming");
  this->streaming_ = false;
  
  if (this->sensor_detected_ && this->csi_handle_) {
    isp_stop(this->isp_handle_);
    esp_cam_ctlr_stop(this->csi_handle_);
  }
  
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
  ESP_LOGCONFIG(TAG, "Tab5 Camera SC202CS:");
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->sensor_detected_ ? "RAW8→ISP→RGB565" : "TEST");
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
    this->frame_buffer_.width, this->frame_buffer_.height);
  ESP_LOGCONFIG(TAG, "  Pipeline: SC202CS(RAW8)→CSI→ISP→RGB565");
  ESP_LOGCONFIG(TAG, "  Buffer: %d bytes @ %p", 
    this->frame_buffer_.length, this->frame_buffer_.buffer);
}

}  // namespace tab5_camera
}  // namespace esphome




