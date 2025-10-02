#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#include "esphome/components/esp32/gpio.h"
#endif

#include <driver/ledc.h>

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Initialisation Tab5 Camera avec driver officiel...");
  
  // 1. Démarrer clock externe
  if (!this->start_external_clock_()) {
    ESP_LOGE(TAG, "Échec clock externe");
    this->mark_failed();
    return;
  }
  
  // 2. Reset matériel si disponible
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(50);
    ESP_LOGI(TAG, "Reset capteur effectué");
  }
  
  // 3. Initialiser avec driver officiel
  if (!this->init_sensor_with_official_driver_()) {
    ESP_LOGE(TAG, "Échec initialisation capteur officiel");
    this->mark_failed();
    return;
  }
  
  // 4. Allouer buffer
  if (!this->allocate_frame_buffer_()) {
    ESP_LOGE(TAG, "Échec allocation buffer");
    this->mark_failed();
    return;
  }
  
  // 5. Initialiser CSI
  if (!this->init_csi_interface_()) {
    ESP_LOGW(TAG, "CSI non disponible, utilisation mode test");
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Caméra Tab5 initialisée avec driver officiel");
}

bool Tab5Camera::init_sensor_with_official_driver_() {
  ESP_LOGI(TAG, "Utilisation du driver esp_cam_sensor officiel...");
  
  // Créer handle SCCB pour I2C existant
  esp_sccb_i2c_config_t i2c_conf = {};
  i2c_conf.scl_speed_hz = 400000;  // 400kHz depuis votre YAML
  i2c_conf.scl_wait_us = 2000;
  i2c_conf.i2c_port = 0;  // I2C bus 0
  i2c_conf.sda_io_num = 31;  // Depuis votre YAML
  i2c_conf.scl_io_num = 32;  // Depuis votre YAML
  
  esp_err_t ret = esp_sccb_new_i2c_bus(&i2c_conf, &this->sccb_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec création bus SCCB: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Configuration pour détection automatique
  esp_cam_sensor_config_t sensor_config = {};
  sensor_config.sccb_handle = this->sccb_handle_;
  sensor_config.reset_pin = -1;  // Géré par nous
  sensor_config.pwdn_pin = -1;
  sensor_config.xclk_pin = 36;  // Depuis votre YAML
  sensor_config.xclk_freq_hz = this->xclk_frequency_;
  sensor_config.sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
  
  // CRITIQUE: Détection automatique avec toute la séquence d'init
  this->sensor_device_ = esp_cam_sensor_detect(&sensor_config);
  
  if (this->sensor_device_ == nullptr) {
    ESP_LOGE(TAG, "Aucun capteur détecté");
    return false;
  }
  
  ESP_LOGI(TAG, "✅ Capteur détecté: %s (PID: 0x%04X)", 
           this->sensor_device_->name, 
           this->sensor_device_->id.pid);
  
  // Configurer le format souhaité
  uint16_t width, height;
  this->get_resolution_dimensions_(width, height);
  
  esp_cam_sensor_format_t desired_format = {};
  desired_format.format = this->convert_pixel_format_();
  desired_format.port = ESP_CAM_SENSOR_MIPI_CSI;
  desired_format.xclk = this->xclk_frequency_;
  desired_format.width = width;
  desired_format.height = height;
  desired_format.fps = this->framerate_;
  
  // Chercher le format supporté le plus proche
  esp_cam_sensor_format_array_t formats = {};
  if (this->sensor_device_->ops->query_support_formats(this->sensor_device_, &formats) == ESP_OK) {
    ESP_LOGI(TAG, "Formats disponibles: %u", formats.count);
    
    // Utiliser le premier format compatible
    for (size_t i = 0; i < formats.count; i++) {
      if (formats.format_array[i].width == width && 
          formats.format_array[i].height == height) {
        ret = this->sensor_device_->ops->set_format(this->sensor_device_, &formats.format_array[i]);
        if (ret == ESP_OK) {
          ESP_LOGI(TAG, "✅ Format configuré: %s", formats.format_array[i].name);
          return true;
        }
      }
    }
  }
  
  // Fallback: essayer de set le format directement
  ret = this->sensor_device_->ops->set_format(this->sensor_device_, &desired_format);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Impossible de définir format exact, utilisation format par défaut");
  }
  
  // CRITIQUE: Désactiver explicitement le test pattern via IOCTL
  int test_pattern_off = 0;
  ret = this->sensor_device_->ops->priv_ioctl(
    this->sensor_device_, 
    ESP_CAM_SENSOR_IOC_S_TEST_PATTERN, 
    &test_pattern_off
  );
  
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "✅ Test pattern désactivé via IOCTL");
  } else {
    ESP_LOGW(TAG, "IOCTL test pattern échoué: %s", esp_err_to_name(ret));
  }
  
  return true;
}

bool Tab5Camera::start_external_clock_() {
  // Votre code LEDC existant fonctionne bien
  if (this->xclk_pin_ == nullptr) {
    return false;
  }
  
  int gpio_num = -1;
  #ifdef USE_ESP32
    auto *esp32_pin = (esphome::esp32::ESP32InternalGPIOPin*)this->xclk_pin_;
    gpio_num = esp32_pin->get_pin();
  #endif
  
  if (gpio_num < 0) {
    return false;
  }
  
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.freq_hz = this->xclk_frequency_;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  
  esp_err_t err = ledc_timer_config(&ledc_timer);
  if (err != ESP_OK) {
    ledc_timer.duty_resolution = LEDC_TIMER_2_BIT;
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
      return false;
    }
  }
  
  ledc_channel_config_t ledc_channel = {};
  ledc_channel.gpio_num = gpio_num;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.duty = (ledc_timer.duty_resolution == LEDC_TIMER_1_BIT) ? 1 : 2;
  
  err = ledc_channel_config(&ledc_channel);
  if (err != ESP_OK) {
    return false;
  }
  
  ESP_LOGI(TAG, "✅ Clock 24MHz démarré sur GPIO%d", gpio_num);
  return true;
}

bool Tab5Camera::allocate_frame_buffer_() {
  uint16_t width, height;
  this->get_resolution_dimensions_(width, height);
  
  size_t buffer_size = width * height * sizeof(uint16_t);  // RGB565
  
  this->frame_buffer_.buffer = (uint8_t *)heap_caps_malloc(
    buffer_size, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (this->frame_buffer_.buffer == nullptr) {
    ESP_LOGE(TAG, "Échec allocation %u bytes", buffer_size);
    return false;
  }
  
  this->frame_buffer_.length = buffer_size;
  this->frame_buffer_.width = width;
  this->frame_buffer_.height = height;
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "✅ Buffer alloué: %ux%u (%u bytes)", width, height, buffer_size);
  return true;
}

bool Tab5Camera::init_csi_interface_() {
  #ifdef CONFIG_ISP_ENABLED
  
  ESP_LOGI(TAG, "Initialisation CSI pour ESP32-P4...");
  
  uint16_t width, height;
  this->get_resolution_dimensions_(width, height);
  
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = width;
  csi_config.v_res = height;
  csi_config.lane_bit_rate_mbps = 576;  // M5Stack utilise 576 Mbps
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 1;  // M5Stack Tab5: 1 lane
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 1;
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_ctlr_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec création contrôleur CSI: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Callbacks
  esp_cam_ctlr_evt_cbs_t cbs = {};
  cbs.on_get_new_trans = [](esp_cam_ctlr_handle_t handle, 
                             esp_cam_ctlr_trans_t *trans, 
                             void *user_data) -> bool {
    Tab5Camera *camera = (Tab5Camera*)user_data;
    trans->buffer = camera->frame_buffer_.buffer;
    trans->buflen = camera->frame_buffer_.length;
    return true;
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->cam_ctlr_handle_, &cbs, this);
  if (ret != ESP_OK) {
    esp_cam_ctlr_del(this->cam_ctlr_handle_);
    return false;
  }
  
  ret = esp_cam_ctlr_enable(this->cam_ctlr_handle_);
  if (ret != ESP_OK) {
    esp_cam_ctlr_del(this->cam_ctlr_handle_);
    return false;
  }
  
  ret = esp_cam_ctlr_start(this->cam_ctlr_handle_);
  if (ret != ESP_OK) {
    esp_cam_ctlr_disable(this->cam_ctlr_handle_);
    esp_cam_ctlr_del(this->cam_ctlr_handle_);
    return false;
  }
  
  this->csi_initialized_ = true;
  ESP_LOGI(TAG, "✅ CSI initialisé: %ux%u, 1 lane @ 576 Mbps", width, height);
  return true;
  
  #else
  ESP_LOGW(TAG, "ISP non activé dans sdkconfig");
  return false;
  #endif
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_ || this->sensor_device_ == nullptr) {
    return false;
  }
  
  // Activer le streaming via driver officiel
  int stream_on = 1;
  esp_err_t ret = this->sensor_device_->ops->priv_ioctl(
    this->sensor_device_,
    ESP_CAM_SENSOR_IOC_S_STREAM,
    &stream_on
  );
  
  if (ret == ESP_OK) {
    this->streaming_ = true;
    ESP_LOGI(TAG, "✅ Streaming démarré");
    return true;
  }
  
  return false;
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) {
    return false;
  }
  
  #ifdef CONFIG_ISP_ENABLED
  if (this->csi_initialized_ && this->cam_ctlr_handle_) {
    return this->capture_csi_frame_();
  }
  #endif
  
  // Fallback: pattern de test
  static uint8_t counter = 0;
  counter++;
  uint16_t *pixels = (uint16_t*)this->frame_buffer_.buffer;
  for (size_t i = 0; i < this->frame_buffer_.width * this->frame_buffer_.height; i++) {
    pixels[i] = (counter << 11) | (counter << 5) | counter;
  }
  return true;
}

bool Tab5Camera::capture_csi_frame_() {
  #ifdef CONFIG_ISP_ENABLED
  esp_cam_ctlr_trans_t trans = {};
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_ctlr_handle_, &trans, 100);
  return (ret == ESP_OK);
  #else
  return false;
  #endif
}

void Tab5Camera::get_resolution_dimensions_(uint16_t &width, uint16_t &height) {
  switch (this->resolution_) {
    case RESOLUTION_1080P: width = 1920; height = 1080; break;
    case RESOLUTION_720P: width = 1280; height = 720; break;
    case RESOLUTION_VGA: width = 640; height = 480; break;
    case RESOLUTION_QVGA: width = 320; height = 240; break;
    default: width = 640; height = 480;
  }
}

esp_cam_sensor_output_format_t Tab5Camera::convert_pixel_format_() {
  switch (this->pixel_format_) {
    case PIXEL_FORMAT_RGB565: return ESP_CAM_SENSOR_PIXFORMAT_RGB565;
    case PIXEL_FORMAT_YUV422: return ESP_CAM_SENSOR_PIXFORMAT_YUV422;
    case PIXEL_FORMAT_RAW8: return ESP_CAM_SENSOR_PIXFORMAT_RAW8;
    case PIXEL_FORMAT_JPEG: return ESP_CAM_SENSOR_PIXFORMAT_JPEG;
    default: return ESP_CAM_SENSOR_PIXFORMAT_RGB565;
  }
}

void Tab5Camera::loop() {}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera (driver officiel):");
  if (this->sensor_device_) {
    ESP_LOGCONFIG(TAG, "  Capteur: %s", this->sensor_device_->name);
    ESP_LOGCONFIG(TAG, "  PID: 0x%04X", this->sensor_device_->id.pid);
  }
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
                this->frame_buffer_.width, this->frame_buffer_.height);
}

}  // namespace tab5_camera
}  // namespace esphome






