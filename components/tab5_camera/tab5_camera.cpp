#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32_VARIANT_ESP32P4
#include "../components/esp_cam_sensor_esphome/esp_cam_sensor_wrapper.h"
using esphome::esp_cam_sensor_esphome::ESPCamSensorWrapper;
#endif

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGI(TAG, "🎥 Initialisation Tab5 Camera");
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  // 1. Init pins
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(20);
  }
  
  if (this->pwdn_pin_ != nullptr) {
    this->pwdn_pin_->setup();
    this->pwdn_pin_->digital_write(false);
  }
  
  // 2. Détecter le capteur SC202CS
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "❌ Échec détection capteur");
    this->mark_failed();
    return;
  }
  
  // 3. Init LDO pour MIPI
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "❌ Échec init LDO");
    this->mark_failed();
    return;
  }
  
  // 4. Init CSI
  if (!this->init_csi_()) {
    ESP_LOGE(TAG, "❌ Échec init CSI");
    this->mark_failed();
    return;
  }
  
  // 5. Init ISP
  if (!this->init_isp_()) {
    ESP_LOGE(TAG, "❌ Échec init ISP");
    this->mark_failed();
    return;
  }
  
  // 6. Allouer le buffer
  if (!this->allocate_buffer_()) {
    ESP_LOGE(TAG, "❌ Échec allocation buffer");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Caméra prête");
  
#else
  ESP_LOGE(TAG, "❌ ESP32-P4 requis");
  this->mark_failed();
#endif
}

#ifdef USE_ESP32_VARIANT_ESP32P4

bool Tab5Camera::init_sensor_() {
  ESP_LOGI(TAG, "Détection SC202CS @ 0x%02X", this->sensor_address_);
  
  i2c_master_bus_handle_t i2c_handle = this->parent_->get_i2c_bus_handle();
  if (!i2c_handle) {
    ESP_LOGE(TAG, "I2C handle invalide");
    return false;
  }
  
  int8_t reset = this->reset_pin_ ? this->reset_pin_->get_pin() : -1;
  int8_t pwdn = this->pwdn_pin_ ? this->pwdn_pin_->get_pin() : -1;
  
  this->sensor_device_ = ESPCamSensorWrapper::detect_sensor(
    i2c_handle, reset, pwdn, this->sensor_address_
  );
  
  if (!this->sensor_device_) {
    ESP_LOGE(TAG, "Capteur non détecté");
    return false;
  }
  
  ESP_LOGI(TAG, "✓ %s détecté (0x%04X)", 
           this->sensor_device_->name, this->sensor_device_->id.pid);
  
  // Configurer le format du capteur
  esp_cam_sensor_format_t format;
  if (ESPCamSensorWrapper::get_format(this->sensor_device_, &format) == ESP_OK) {
    ESP_LOGI(TAG, "  Format: %ux%u @ %ufps", format.width, format.height, format.fps);
  }
  
  return true;
}

bool Tab5Camera::init_ldo_() {
  ESP_LOGI(TAG, "Init LDO MIPI");
  
  esp_ldo_channel_config_t ldo_config = {
    .chan_id = 3,
    .voltage_mv = 2500,
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_config, &this->ldo_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LDO failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ LDO OK (2.5V)");
  return true;
}

bool Tab5Camera::init_csi_() {
  ESP_LOGI(TAG, "Init MIPI-CSI");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  esp_cam_ctlr_csi_config_t csi_config = {
    .ctlr_id = 0,
    .clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT,
    .byte_swap_en = false,
    .queue_items = 3,
    .h_res = res.width,
    .v_res = res.height,
    .data_lane_num = 1,
    .input_data_color_type = CAM_CTLR_COLOR_RAW8,
    .output_data_color_type = CAM_CTLR_COLOR_RGB565,
    .lane_bit_rate_mbps = 576,
  };
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI failed: %d", ret);
    return false;
  }
  
  // Callbacks
  esp_cam_ctlr_evt_cbs_t callbacks = {
    .on_get_new_trans = Tab5Camera::on_csi_new_frame_,
    .on_trans_finished = Tab5Camera::on_csi_frame_done_,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->csi_handle_, &callbacks, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Callbacks failed: %d", ret);
    return false;
  }
  
  ret = esp_cam_ctlr_enable(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Enable CSI failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ CSI OK (%ux%u)", res.width, res.height);
  return true;
}

bool Tab5Camera::init_isp_() {
  ESP_LOGI(TAG, "Init ISP");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  esp_isp_processor_cfg_t isp_config = {
    .clk_src = ISP_CLK_SRC_DEFAULT,
    .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
    .has_line_start_packet = false,
    .has_line_end_packet = false,
    .h_res = res.width,
    .v_res = res.height,
    .clk_hz = 80000000,
    .input_data_color_type = ISP_COLOR_RAW8,
    .output_data_color_type = ISP_COLOR_RGB565,
  };
  
  esp_err_t ret = esp_isp_new_processor(&isp_config, &this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP create failed: %d", ret);
    return false;
  }
  
  ret = esp_isp_enable(this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP enable failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ ISP OK");
  return true;
}

bool Tab5Camera::allocate_buffer_() {
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_size_ = res.width * res.height * 2; // RGB565
  
  this->frame_buffers_[0] = (uint8_t*)heap_caps_aligned_alloc(
    64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM
  );
  
  this->frame_buffers_[1] = (uint8_t*)heap_caps_aligned_alloc(
    64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM
  );
  
  if (!this->frame_buffers_[0] || !this->frame_buffers_[1]) {
    ESP_LOGE(TAG, "Buffer alloc failed");
    return false;
  }
  
  this->current_frame_buffer_ = this->frame_buffers_[0];
  
  ESP_LOGI(TAG, "✓ Buffers: 2x%u bytes", this->frame_buffer_size_);
  return true;
}

bool IRAM_ATTR Tab5Camera::on_csi_new_frame_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  Tab5Camera *cam = (Tab5Camera*)user_data;
  
  // Donner le prochain buffer disponible
  trans->buffer = cam->frame_buffers_[cam->buffer_index_];
  trans->buflen = cam->frame_buffer_size_;
  
  return false;
}

bool IRAM_ATTR Tab5Camera::on_csi_frame_done_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  Tab5Camera *cam = (Tab5Camera*)user_data;
  
  if (trans->received_size > 0) {
    // Basculer vers le buffer suivant
    cam->buffer_index_ = (cam->buffer_index_ + 1) % 2;
    cam->current_frame_buffer_ = cam->frame_buffers_[cam->buffer_index_];
    cam->frame_ready_ = true;
  }
  
  return false;
}

CameraResolutionInfo Tab5Camera::get_resolution_info_() {
  switch (this->resolution_) {
    case RESOLUTION_720P: return {1280, 720};
    case RESOLUTION_1080P: return {1920, 1080};
    case RESOLUTION_QVGA: return {320, 240};
    case RESOLUTION_VGA:
    default: return {640, 480};
  }
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_ || this->streaming_) {
    return false;
  }
  
  ESP_LOGI(TAG, "Démarrage streaming");
  
  // Démarrer le capteur
  esp_err_t ret = ESPCamSensorWrapper::start_stream(this->sensor_device_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Start sensor failed: %d", ret);
    return false;
  }
  
  // Démarrer CSI
  ret = esp_cam_ctlr_start(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Start CSI failed: %d", ret);
    ESPCamSensorWrapper::stop_stream(this->sensor_device_);
    return false;
  }
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "✅ Streaming actif");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_) {
    return true;
  }
  
  esp_cam_ctlr_stop(this->csi_handle_);
  ESPCamSensorWrapper::stop_stream(this->sensor_device_);
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "⏹ Streaming arrêté");
  return true;
}

bool Tab5Camera::capture_frame() {
  if (!this->streaming_ || !this->frame_ready_) {
    return false;
  }
  
  this->frame_ready_ = false;
  return true;
}

uint16_t Tab5Camera::get_image_width() const {
  return this->get_resolution_info_().width;
}

uint16_t Tab5Camera::get_image_height() const {
  return this->get_resolution_info_().height;
}

#endif  // USE_ESP32_VARIANT_ESP32P4

void Tab5Camera::loop() {
  // Rien - tout est géré par les callbacks ISR
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Capteur: SC202CS @ 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
                this->get_image_width(), this->get_image_height());
  ESP_LOGCONFIG(TAG, "  Format: RGB565");
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_ ? "OUI" : "NON");
  ESP_LOGCONFIG(TAG, "  Initialisé: %s", this->initialized_ ? "OUI" : "NON");
}

}  // namespace tab5_camera
}  // namespace esphome



