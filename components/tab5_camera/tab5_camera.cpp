#include "tab5_camera.h"
#include "esphome/core/log.h"

// Headers ESP-IDF pour CSI/ISP (via CMakeLists.txt)
#ifdef USE_ESP32_VARIANT_ESP32P4
  #include "esp_cam_ctlr.h"
  #include "esp_cam_ctlr_csi.h"
  #include "driver/isp.h"
  #include "driver/isp_core.h"
  #include "driver/isp_bf.h"
  #include "driver/isp_color.h"
#endif

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Initialisation Tab5 Camera (CSI/ISP)...");
  
  // Buffer RGB565
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
  
  ESP_LOGI(TAG, "Buffer: %ux%u = %u bytes @ %p", 
           res.width, res.height, this->frame_buffer_.length, this->frame_buffer_.buffer);
  
  // Pattern de test initial
  this->init_test_pattern_();
  
  // Détecter SC202CS
  uint16_t chip_id = this->read_chip_id_();
  if (chip_id == SC2356_CHIP_ID_VALUE) {
    ESP_LOGI(TAG, "SC202CS détecté: 0x%04X", chip_id);
    this->sensor_detected_ = true;
    
#ifdef USE_ESP32_VARIANT_ESP32P4
    // Initialiser CSI/ISP
    if (this->init_csi_isp_()) {
      ESP_LOGI(TAG, "Pipeline CSI→ISP initialisé");
    } else {
      ESP_LOGW(TAG, "Échec init pipeline - mode test");
    }
#endif
  } else {
    ESP_LOGW(TAG, "Chip ID: 0x%04X (mode test)", chip_id);
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "Tab5 Camera prête");
}

uint16_t Tab5Camera::read_chip_id_() {
  uint8_t id_h = 0, id_l = 0;
  uint8_t reg_buf[2];
  
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

#ifdef USE_ESP32_VARIANT_ESP32P4
bool Tab5Camera::init_csi_isp_() {
  ESP_LOGI(TAG, "Init CSI/ISP pour SC202CS...");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  // Configuration CSI
  esp_cam_ctlr_csi_config_t csi_config = {
    .ctlr_id = 0,
    .h_res = res.width,
    .v_res = res.height,
    .lane_bit_rate_mbps = 576,  // SC202CS: 576 Mbps
    .input_data_color_type = CAM_CTLR_COLOR_RAW8,
    .output_data_color_type = CAM_CTLR_COLOR_RAW8,
    .data_lane_num = 1,  // 1 lane MIPI
    .byte_swap_en = false,
    .queue_items = 2,
  };
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI init failed: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Configuration ISP (RAW8 BGGR → RGB565)
  isp_proc_handle_t isp_proc = nullptr;
  isp_processor_cfg_t isp_config = {
    .clk_hz = 120 * 1000 * 1000,
    .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
    .input_data_color_format = ISP_COLOR_RAW8,
    .output_data_color_format = ISP_COLOR_RGB565,
    .has_line_start_packet = false,
    .has_line_end_packet = false,
    .h_res = res.width,
    .v_res = res.height,
    .bayer_type = ISP_BAYER_BGGR,
  };
  
  ret = isp_new_processor(&isp_config, &isp_proc);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP init failed: %s", esp_err_to_name(ret));
    return false;
  }
  
  this->isp_handle_ = isp_proc;
  
  // Configurer le SC202CS pour RAW8
  this->configure_sc202cs_();
  
  // Activer le pipeline
  isp_enable(this->isp_handle_);
  esp_cam_ctlr_enable(this->csi_handle_);
  
  ESP_LOGI(TAG, "Pipeline actif: CSI(RAW8)→ISP→RGB565");
  return true;
}

void Tab5Camera::configure_sc202cs_() {
  ESP_LOGI(TAG, "Configuration SC202CS...");
  
  // Reset
  uint8_t cmd[3] = {0x01, 0x03, 0x01};
  this->write(cmd, 3);
  delay(10);
  
  // Mode streaming RAW8
  cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x01;
  this->write(cmd, 3);
  
  ESP_LOGI(TAG, "SC202CS configuré");
}
#endif

void Tab5Camera::init_test_pattern_() {
  CameraResolutionInfo res = this->get_resolution_info_();
  
  for (size_t y = 0; y < res.height; y++) {
    uint16_t color;
    if (y < res.height / 3) {
      color = 0xF800;
    } else if (y < 2 * res.height / 3) {
      color = 0x07E0;
    } else {
      color = 0x001F;
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
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  if (this->sensor_detected_ && this->csi_handle_) {
    // TODO: Recevoir frame depuis ISP
    // Pour l'instant pattern de test
  }
#endif
  
  // Animation test
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
  ESP_LOGI(TAG, "Streaming started");
  this->streaming_ = true;
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  if (this->csi_handle_) {
    esp_cam_ctlr_start(this->csi_handle_);
  }
#endif
  
  return true;
}

bool Tab5Camera::stop_streaming() {
  ESP_LOGI(TAG, "Streaming stopped");
  this->streaming_ = false;
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  if (this->csi_handle_) {
    esp_cam_ctlr_stop(this->csi_handle_);
  }
#endif
  
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
  ESP_LOGCONFIG(TAG, "  SC202CS: %s", this->sensor_detected_ ? "Détecté" : "Test");
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
    this->frame_buffer_.width, this->frame_buffer_.height);
}

}  // namespace tab5_camera
}  // namespace esphome



