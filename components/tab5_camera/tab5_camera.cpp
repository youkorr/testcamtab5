#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32_VARIANT_ESP32P4
#include "../esp_cam_sensor_esphome/esp_cam_sensor_wrapper.h"
using esphome::esp_cam_sensor_esphome::ESPCamSensorWrapper;
#endif

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "🎥 Initialisation Tab5 Camera...");
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  // 1. Initialiser les pins
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
  
  // 2. Détecter le capteur
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "❌ Échec détection capteur");
    this->mark_failed();
    return;
  }
  
  // 3. Initialiser CSI
  if (!this->init_csi_()) {
    ESP_LOGE(TAG, "❌ Échec initialisation CSI");
    this->mark_failed();
    return;
  }
  
  // 4. Initialiser ISP
  if (!this->init_isp_()) {
    ESP_LOGE(TAG, "❌ Échec initialisation ISP");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Caméra initialisée avec succès");
  
#else
  ESP_LOGE(TAG, "❌ ESP32-P4 requis");
  this->mark_failed();
#endif
}

#ifdef USE_ESP32_VARIANT_ESP32P4

bool Tab5Camera::init_sensor_() {
  ESP_LOGI(TAG, "Détection capteur SC202CS à l'adresse 0x%02X...", this->sensor_address_);
  
  // Obtenir le handle I2C d'ESPHome
  i2c_master_bus_handle_t i2c_handle = this->parent_->get_i2c_bus_handle();
  if (i2c_handle == nullptr) {
    ESP_LOGE(TAG, "Handle I2C invalide");
    return false;
  }
  
  // Détection du capteur
  int8_t reset = this->reset_pin_ ? this->reset_pin_->get_pin() : -1;
  int8_t pwdn = this->pwdn_pin_ ? this->pwdn_pin_->get_pin() : -1;
  
  this->sensor_device_ = ESPCamSensorWrapper::detect_sensor(
    i2c_handle, reset, pwdn, this->sensor_address_
  );
  
  if (this->sensor_device_ == nullptr) {
    ESP_LOGE(TAG, "Aucun capteur détecté");
    return false;
  }
  
  ESP_LOGI(TAG, "✓ Capteur détecté: %s (ID: 0x%04X)", 
           this->sensor_device_->name, this->sensor_device_->id.pid);
  
  // Vérifier l'ID du SC202CS
  if (this->sensor_device_->id.pid != 0xEB52) {
    ESP_LOGW(TAG, "ID capteur inattendu: 0x%04X (attendu: 0xEB52)", 
             this->sensor_device_->id.pid);
  }
  
  // Configurer le format
  esp_cam_sensor_format_t format;
  esp_err_t ret = ESPCamSensorWrapper::get_format(this->sensor_device_, &format);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec lecture format: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "Format capteur: %ux%u @ %u fps", 
           format.width, format.height, format.fps);
  
  return true;
}

bool Tab5Camera::init_csi_() {
  ESP_LOGI(TAG, "Initialisation MIPI-CSI...");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  esp_cam_ctlr_csi_config_t csi_config = {
    .ctlr_id = 0,
    .clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT,
    .byte_swap_en = false,
    .queue_items = 1,
    .h_res = res.width,
    .v_res = res.height,
    .data_lane_num = 1,
    .input_data_color_type = CAM_CTLR_COLOR_RAW8,
    .output_data_color_type = CAM_CTLR_COLOR_RGB565,
    .lane_bit_rate_mbps = 576,
  };
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec création CSI: %d", ret);
    return false;
  }
  
  esp_cam_ctlr_evt_cbs_t callbacks = {
    .on_trans_finished = Tab5Camera::on_csi_frame_done_,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->csi_handle_, &callbacks, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec enregistrement callbacks: %d", ret);
    return false;
  }
  
  ret = esp_cam_ctlr_enable(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec activation CSI: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ CSI initialisé");
  return true;
}

bool Tab5Camera::init_isp_() {
  ESP_LOGI(TAG, "Initialisation ISP...");
  
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
    ESP_LOGE(TAG, "Échec création ISP: %d", ret);
    return false;
  }
  
  ret = esp_isp_enable(this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec activation ISP: %d", ret);
    return false;
  }
  
  // Allouer le buffer
  this->frame_buffer_size_ = res.width * res.height * 2;  // RGB565 = 2 bytes/pixel
  this->current_frame_buffer_ = (uint8_t*)heap_caps_malloc(
    this->frame_buffer_size_,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (this->current_frame_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Échec allocation buffer");
    return false;
  }
  
  ESP_LOGI(TAG, "✓ ISP initialisé (buffer: %u bytes)", this->frame_buffer_size_);
  return true;
}

bool IRAM_ATTR Tab5Camera::on_csi_frame_done_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  
  if (trans->received_size > 0 && camera->current_frame_buffer_ != nullptr) {
    memcpy(camera->current_frame_buffer_, trans->buffer, 
           MIN(trans->received_size, camera->frame_buffer_size_));
  }
  
  return true;
}

CameraResolutionInfo Tab5Camera::get_resolution_info_() {
  switch (this->resolution_) {
    case RESOLUTION_720P:
      return {1280, 720};
    case RESOLUTION_1080P:
      return {1920, 1080};
    case RESOLUTION_VGA:
    default:
      return {640, 480};
  }
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_ || this->streaming_) {
    return false;
  }
  
  esp_err_t ret = esp_cam_ctlr_start(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec démarrage CSI: %d", ret);
    return false;
  }
  
  ret = ESPCamSensorWrapper::start_stream(this->sensor_device_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec démarrage capteur: %d", ret);
    return false;
  }
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "✓ Streaming démarré");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_) {
    return false;
  }
  
  ESPCamSensorWrapper::stop_stream(this->sensor_device_);
  esp_cam_ctlr_stop(this->csi_handle_);
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "✓ Streaming arrêté");
  return true;
}

bool Tab5Camera::capture_frame() {
  return this->streaming_;
}

uint16_t Tab5Camera::get_image_width() const {
  CameraResolution res_enum = this->resolution_;
  switch (res_enum) {
    case RESOLUTION_720P: return 1280;
    case RESOLUTION_1080P: return 1920;
    default: return 640;
  }
}

uint16_t Tab5Camera::get_image_height() const {
  CameraResolution res_enum = this->resolution_;
  switch (res_enum) {
    case RESOLUTION_720P: return 720;
    case RESOLUTION_1080P: return 1080;
    default: return 480;
  }
}

#endif  // USE_ESP32_VARIANT_ESP32P4

void Tab5Camera::loop() {
  // Rien à faire ici, tout est géré par les callbacks ISR
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Capteur: SC202CS @ 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", this->get_image_width(), this->get_image_height());
  ESP_LOGCONFIG(TAG, "  Format: %s", this->pixel_format_ == PIXEL_FORMAT_RGB565 ? "RGB565" : "RAW8");
  ESP_LOGCONFIG(TAG, "  Framerate: %u FPS", this->framerate_);
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_ ? "OUI" : "NON");
}

}  // namespace tab5_camera
}  // namespace esphome



