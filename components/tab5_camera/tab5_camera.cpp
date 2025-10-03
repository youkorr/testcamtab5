#include "tab5_camera.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32_VARIANT_ESP32P4
extern "C" {
  #include "esp_cam_sensor.h"
  #include "esp_cam_sensor_detect.h"
}
#endif

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Initialisation Tab5 Camera (driver Espressif)...");
  
  // Allouer buffer RGB565 dans PSRAM
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_.width = res.width;
  this->frame_buffer_.height = res.height;
  this->frame_buffer_.length = res.width * res.height * 2;
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "Résolution: %ux%u", res.width, res.height);
  ESP_LOGI(TAG, "Buffer: %u bytes", this->frame_buffer_.length);
  
  this->frame_buffer_.buffer = (uint8_t*)heap_caps_malloc(
    this->frame_buffer_.length, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (!this->frame_buffer_.buffer) {
    ESP_LOGE(TAG, "Échec allocation buffer");
    this->mark_failed();
    return;
  }
  
  ESP_LOGI(TAG, "Buffer alloué: %p", this->frame_buffer_.buffer);
  
  // Initialiser pattern de test
  this->init_test_pattern_();
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  // Initialiser le capteur avec le driver Espressif
  if (this->init_espressif_sensor_()) {
    ESP_LOGI(TAG, "SC202CS initialisé via driver Espressif");
    this->sensor_detected_ = true;
  } else {
    ESP_LOGW(TAG, "Échec init capteur - mode test");
  }
#else
  ESP_LOGW(TAG, "ESP32-P4 requis pour le capteur");
#endif
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "Tab5 Camera prête");
}

#ifdef USE_ESP32_VARIANT_ESP32P4
bool Tab5Camera::init_espressif_sensor_() {
  ESP_LOGI(TAG, "Init SC202CS avec driver Espressif...");
  
  // Configuration SCCB (I2C)
  esp_sccb_io_handle_t sccb_handle = nullptr;
  
  // Utiliser le bus I2C d'ESPHome
  // Note: Il faudra créer le handle SCCB depuis le bus I2C ESPHome
  // Pour l'instant, on utilise une approche simplifiée
  
  ESP_LOGI(TAG, "Création handle SCCB...");
  i2c_master_bus_config_t i2c_bus_conf = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 31,  // GPIO depuis votre config
    .scl_io_num = 32,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags = {
      .enable_internal_pullup = 1,
    },
  };
  
  i2c_master_bus_handle_t i2c_bus = nullptr;
  if (i2c_new_master_bus(&i2c_bus_conf, &i2c_bus) != ESP_OK) {
    ESP_LOGE(TAG, "Échec création bus I2C");
    return false;
  }
  
  // Créer handle SCCB depuis I2C
  sccb_i2c_config_t sccb_conf = {
    .scl_speed_hz = 100000,  // 100kHz pour SCCB
    .device_address = this->sensor_address_,
  };
  
  if (sccb_new_i2c_io(i2c_bus, &sccb_conf, &sccb_handle) != ESP_OK) {
    ESP_LOGE(TAG, "Échec création SCCB handle");
    return false;
  }
  
  // Configuration du capteur
  esp_cam_sensor_config_t cam_config = {
    .sccb_handle = sccb_handle,
    .reset_pin = -1,  // Géré par GPIO expander
    .pwdn_pin = -1,
    .xclk_pin = (int8_t)this->xclk_pin_->get_pin(),
    .xclk_freq_hz = (int32_t)this->xclk_frequency_,
    .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
  };
  
  // Détecter le capteur (utilise sc202cs_detect du driver)
  this->sensor_device_ = sc202cs_detect(&cam_config);
  
  if (!this->sensor_device_) {
    ESP_LOGE(TAG, "SC202CS non détecté");
    return false;
  }
  
  ESP_LOGI(TAG, "SC202CS détecté: PID=0x%04X", this->sensor_device_->id.pid);
  
  // Configurer le format (RAW8 720p pour commencer)
  esp_cam_sensor_format_t format;
  if (this->sensor_device_->ops->get_format(this->sensor_device_, &format) == ESP_OK) {
    ESP_LOGI(TAG, "Format: %s (%ux%u @ %dfps)", 
             format.name, format.width, format.height, format.fps);
  }
  
  // Initialiser CSI + ISP
  if (!this->init_csi_isp_()) {
    ESP_LOGE(TAG, "Échec init CSI/ISP");
    return false;
  }
  
  return true;
}

bool Tab5Camera::init_csi_isp_() {
  ESP_LOGI(TAG, "Init pipeline CSI→ISP...");
  
  // Récupérer le format actuel
  esp_cam_sensor_format_t format;
  this->sensor_device_->ops->get_format(this->sensor_device_, &format);
  
  // Configuration CSI
  esp_cam_ctlr_csi_config_t csi_config = {
    .ctlr_id = 0,
    .h_res = format.width,
    .v_res = format.height,
    .lane_bit_rate_mbps = (uint32_t)(format.mipi_info.mipi_clk / 1000000),
    .input_data_color_type = ESP_CAM_CTLR_COLOR_RAW8,
    .output_data_color_type = ESP_CAM_CTLR_COLOR_RAW8,
    .data_lane_num = format.mipi_info.lane_num,
    .byte_swap_en = false,
    .queue_items = 2,
  };
  
  if (esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_) != ESP_OK) {
    ESP_LOGE(TAG, "Échec init CSI");
    return false;
  }
  
  // Configuration ISP (RAW8 → RGB565)
  isp_processor_cfg_t isp_config = {
    .clk_hz = 120000000,
    .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
    .input_data_color_format = ISP_COLOR_RAW8,
    .output_data_color_format = ISP_COLOR_RGB565,
    .has_line_start_packet = false,
    .has_line_end_packet = false,
    .h_res = format.width,
    .v_res = format.height,
    .bayer_type = ISP_BAYER_BGGR,  // Bayer pattern du SC202CS
  };
  
  if (isp_new_processor(&isp_config, &this->isp_handle_) != ESP_OK) {
    ESP_LOGE(TAG, "Échec init ISP");
    return false;
  }
  
  // Callback ISP
  isp_evt_cbs_t isp_cbs = {
    .on_frame_done = nullptr,  // Polling mode pour commencer
  };
  
  isp_register_event_callbacks(this->isp_handle_, &isp_cbs, this);
  
  // Activer pipeline
  isp_enable(this->isp_handle_);
  esp_cam_ctlr_enable(this->csi_handle_);
  
  ESP_LOGI(TAG, "Pipeline CSI→ISP actif");
  return true;
}
#endif

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
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  if (this->sensor_detected_ && this->isp_handle_) {
    // TODO: Capturer depuis ISP en polling
    // Pour l'instant, continuer avec test pattern
    ESP_LOGV(TAG, "Capture ISP à implémenter");
  }
#endif
  
  // Mode test: animer
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
  ESP_LOGI(TAG, "Démarrage streaming");
  this->streaming_ = true;
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  if (this->sensor_device_) {
    // Démarrer le stream du capteur
    int enable = 1;
    esp_cam_sensor_ioctl(this->sensor_device_, ESP_CAM_SENSOR_IOC_S_STREAM, &enable);
    
    if (this->csi_handle_) {
      esp_cam_ctlr_start(this->csi_handle_);
    }
  }
#endif
  
  return true;
}

bool Tab5Camera::stop_streaming() {
  ESP_LOGI(TAG, "Arrêt streaming");
  this->streaming_ = false;
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  if (this->sensor_device_) {
    int enable = 0;
    esp_cam_sensor_ioctl(this->sensor_device_, ESP_CAM_SENSOR_IOC_S_STREAM, &enable);
    
    if (this->csi_handle_) {
      esp_cam_ctlr_stop(this->csi_handle_);
    }
  }
#endif
  
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
  ESP_LOGCONFIG(TAG, "Tab5 Camera (Espressif driver):");
  ESP_LOGCONFIG(TAG, "  Capteur: %s", this->sensor_detected_ ? "SC202CS" : "Test");
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
    this->frame_buffer_.width, this->frame_buffer_.height);
  ESP_LOGCONFIG(TAG, "  I2C: 0x%02X", this->sensor_address_);
}

}  // namespace tab5_camera
}  // namespace esphome



