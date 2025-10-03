#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32_VARIANT_ESP32P4
#include "tab5_camera_sensor.h"

// Inclure l'implémentation des fonctions sensor
#include "tab5_camera_sensor_impl.cpp"

// Inline wrapper functions
namespace {
  esp_cam_sensor_device_t* detect_sensor_inline(
    i2c_master_bus_handle_t i2c_handle,
    int8_t reset_pin,
    int8_t pwdn_pin,
    uint8_t sccb_addr
  ) {
    esp_sccb_io_handle_t sccb_handle;
    sccb_i2c_config_t sccb_config = {};
    sccb_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    sccb_config.device_address = sccb_addr;
    sccb_config.scl_speed_hz = 400000;
    
    esp_err_t ret = sccb_new_i2c_io(i2c_handle, &sccb_config, &sccb_handle);
    if (ret != ESP_OK) {
      return nullptr;
    }
    
    esp_cam_sensor_config_t sensor_config = {};
    sensor_config.sccb_handle = sccb_handle;
    sensor_config.reset_pin = reset_pin;
    sensor_config.pwdn_pin = pwdn_pin;
    sensor_config.xclk_pin = -1;
    sensor_config.xclk_freq_hz = 24000000;
    sensor_config.sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    
    // Parcourir les fonctions de détection
    // Note: __esp_cam_sensor_detect_fn_array_start/end sont définis comme weak
    // donc ils peuvent être NULL si sc202cs n'est pas linké
    if (&__esp_cam_sensor_detect_fn_array_start == &__esp_cam_sensor_detect_fn_array_end) {
      // Pas de capteurs enregistrés
      return nullptr;
    }
    
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
      
      if (p->port == ESP_CAM_SENSOR_MIPI_CSI && p->sccb_addr == sccb_addr) {
        esp_cam_sensor_device_t *dev = p->detect(&sensor_config);
        if (dev != nullptr) {
          return dev;
        }
      }
    }
    
    return nullptr;
  }
}
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
  
  // Créer un bus I2C pour communiquer avec le capteur
  i2c_master_bus_config_t i2c_bus_config = {};
  i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_bus_config.i2c_port = I2C_NUM_0;
  i2c_bus_config.scl_io_num = static_cast<gpio_num_t>(this->i2c_scl_pin_);
  i2c_bus_config.sda_io_num = static_cast<gpio_num_t>(this->i2c_sda_pin_);
  i2c_bus_config.glitch_ignore_cnt = 7;
  i2c_bus_config.flags.enable_internal_pullup = true;
  
  ESP_LOGI(TAG, "I2C config: SCL=%d, SDA=%d", this->i2c_scl_pin_, this->i2c_sda_pin_);
  
  i2c_master_bus_handle_t i2c_handle;
  esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &i2c_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C bus init failed: %d", ret);
    return false;
  }
  
  // Les pins reset/pwdn sont gérées en dehors du sensor driver
  int8_t reset = -1;
  int8_t pwdn = -1;
  
  this->sensor_device_ = detect_sensor_inline(
    i2c_handle, reset, pwdn, this->sensor_address_
  );
  
  if (!this->sensor_device_) {
    ESP_LOGE(TAG, "Capteur non détecté");
    i2c_del_master_bus(i2c_handle);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ %s détecté (0x%04X)", 
           this->sensor_device_->name, this->sensor_device_->id.pid);
  
  // Configurer le format du capteur
  esp_cam_sensor_format_t format;
  if (esp_cam_sensor_get_format(this->sensor_device_, &format) == ESP_OK) {
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
  
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT;
  csi_config.h_res = res.width;
  csi_config.v_res = res.height;
  csi_config.lane_bit_rate_mbps = 576;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 1;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 3;
  
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
  
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_src = ISP_CLK_SRC_DEFAULT;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.h_res = res.width;
  isp_config.v_res = res.height;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.clk_hz = 80000000;
  
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

CameraResolutionInfo Tab5Camera::get_resolution_info_() const {
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
  
  // Le capteur devrait déjà être configuré via les registres par défaut
  // On démarre juste le CSI controller
  esp_err_t ret = esp_cam_ctlr_start(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Start CSI failed: %d", ret);
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
  ESP_LOGCONFIG(TAG, "  I2C: SCL=%d, SDA=%d", this->i2c_scl_pin_, this->i2c_sda_pin_);
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
                this->get_image_width(), this->get_image_height());
  ESP_LOGCONFIG(TAG, "  Format: RGB565");
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_ ? "OUI" : "NON");
  ESP_LOGCONFIG(TAG, "  Initialisé: %s", this->initialized_ ? "OUI" : "NON");
}

}  // namespace tab5_camera
}  // namespace esphome


