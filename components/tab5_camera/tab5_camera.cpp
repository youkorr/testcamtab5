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

// Tables de configuration SC202CS pour différentes résolutions
static const uint16_t SC202CS_1080P_REGS[][2] = { /* ... remplir avec les valeurs existantes ... */ };
static const uint16_t SC202CS_720P_REGS[][2]  = { /* ... */ };
static const uint16_t SC202CS_VGA_REGS[][2]   = { /* ... */ };
static const uint16_t SC202CS_QVGA_REGS[][2]  = { /* ... */ };

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Configuration Tab5 Camera...");

  if (!this->start_external_clock_()) {
    ESP_LOGE(TAG, "Échec de l'initialisation du clock externe");
    this->mark_failed();
    return;
  }

  if (this->reset_pin_ != nullptr) {
    if (!this->reset_sensor_()) {
      ESP_LOGE(TAG, "Échec du reset du capteur");
      this->mark_failed();
      return;
    }
  }

  delay(50);

  if (!this->init_sc202cs_sensor_()) {
    ESP_LOGE(TAG, "Échec de l'initialisation du capteur SC202CS");
    this->mark_failed();
    return;
  }

  if (!this->configure_sc202cs_()) {
    ESP_LOGE(TAG, "Échec de la configuration du capteur");
    this->mark_failed();
    return;
  }

  if (!this->allocate_frame_buffer_()) {
    ESP_LOGE(TAG, "Échec de l'allocation du buffer");
    this->mark_failed();
    return;
  }

  if (!this->init_csi_interface_()) {
    ESP_LOGW(TAG, "CSI non initialisé, la capture pourra échouer");
  }

  this->initialized_ = true;
  ESP_LOGI(TAG, "Caméra Tab5 initialisée avec succès");
}

void Tab5Camera::loop() {
  // Pas de tâches périodiques pour l'instant
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Nom: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Adresse I2C capteur: 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Fréquence clock: %u Hz", this->xclk_frequency_);

  CameraResolutionInfo res_info = this->get_resolution_info_();
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", res_info.width, res_info.height);

  const char *format_str = "Unknown";
  switch (this->pixel_format_) {
    case PIXEL_FORMAT_RGB565: format_str = "RGB565"; break;
    case PIXEL_FORMAT_YUV422: format_str = "YUV422"; break;
    case PIXEL_FORMAT_RAW8: format_str = "RAW8"; break;
    case PIXEL_FORMAT_JPEG: format_str = "JPEG"; break;
  }
  ESP_LOGCONFIG(TAG, "  Format pixel: %s", format_str);
  ESP_LOGCONFIG(TAG, "  Qualité JPEG: %u", this->jpeg_quality_);
  ESP_LOGCONFIG(TAG, "  Framerate: %u fps", this->framerate_);

  ESP_LOGCONFIG(TAG, "  État: %s", this->initialized_ ? "Initialisé" : "Non initialisé");
}

bool Tab5Camera::start_external_clock_() {
  if (!this->xclk_pin_) {
    ESP_LOGE(TAG, "Pin clock externe non configuré");
    return false;
  }

#ifdef USE_ESP32
  auto *esp32_pin = (esphome::esp32::ESP32InternalGPIOPin*)this->xclk_pin_;
  int gpio_num = esp32_pin->get_pin();
#else
  ESP_LOGE(TAG, "Plateforme non ESP32");
  return false;
#endif

  if (gpio_num < 0) {
    ESP_LOGE(TAG, "Numéro de GPIO invalide");
    return false;
  }

  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.freq_hz = this->xclk_frequency_;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer.deconfigure = false;

  if (ledc_timer_config(&ledc_timer) != ESP_OK) {
    ESP_LOGE(TAG, "Échec configuration timer LEDC");
    return false;
  }

  ledc_channel_config_t ledc_channel = {};
  ledc_channel.gpio_num = gpio_num;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.duty = 1;
  ledc_channel.hpoint = 0;
  ledc_channel.flags.output_invert = 0;

  if (ledc_channel_config(&ledc_channel) != ESP_OK) {
    ESP_LOGE(TAG, "Échec configuration canal LEDC");
    return false;
  }

  ESP_LOGI(TAG, "Clock externe démarré à %u Hz sur GPIO%d", this->xclk_frequency_, gpio_num);
  return true;
}

bool Tab5Camera::reset_sensor_() {
  if (!this->reset_pin_) return true;

  this->reset_pin_->setup();
  this->reset_pin_->digital_write(false);
  delay(10);
  this->reset_pin_->digital_write(true);
  delay(10);

  ESP_LOGI(TAG, "Reset du capteur effectué");
  return true;
}

bool Tab5Camera::init_sc202cs_sensor_() {
  uint8_t id_high, id_low;
  if (!this->read_sensor_reg_(SC202CS_CHIP_ID_REG, id_high)) return false;
  if (!this->read_sensor_reg_(SC202CS_CHIP_ID_REG+1, id_low)) return false;

  uint16_t chip_id = (id_high << 8) | id_low;
  ESP_LOGI(TAG, "Chip ID détecté: 0x%04X", chip_id);

  if (chip_id != SC202CS_CHIP_ID_VALUE && chip_id != SC2356_CHIP_ID_VALUE) {
    ESP_LOGW(TAG, "Chip ID inattendu: 0x%04X", chip_id);
  }
  return true;
}

bool Tab5Camera::configure_sc202cs_() {
  if (!this->write_sensor_reg_(SC202CS_RESET_REG, 0x01)) return false;
  delay(100);

  const uint16_t (*regs)[2] = nullptr;
  size_t reg_count = 0;

  switch (this->resolution_) {
    case RESOLUTION_1080P: regs = SC202CS_1080P_REGS; reg_count = sizeof(SC202CS_1080P_REGS)/sizeof(SC202CS_1080P_REGS[0]); break;
    case RESOLUTION_720P:  regs = SC202CS_720P_REGS;  reg_count = sizeof(SC202CS_720P_REGS)/sizeof(SC202CS_720P_REGS[0]); break;
    case RESOLUTION_VGA:   regs = SC202CS_VGA_REGS;   reg_count = sizeof(SC202CS_VGA_REGS)/sizeof(SC202CS_VGA_REGS[0]); break;
    case RESOLUTION_QVGA:  regs = SC202CS_QVGA_REGS;  reg_count = sizeof(SC202CS_QVGA_REGS)/sizeof(SC202CS_QVGA_REGS[0]); break;
  }

  return this->write_sensor_regs_(regs, reg_count);
}

bool Tab5Camera::write_sensor_reg_(uint16_t reg, uint8_t value) {
  uint8_t data[3] = { uint8_t(reg>>8), uint8_t(reg & 0xFF), value };
  return this->write(data, 3) == i2c::ERROR_OK;
}

bool Tab5Camera::read_sensor_reg_(uint16_t reg, uint8_t &value) {
  uint8_t reg_data[2] = { uint8_t(reg>>8), uint8_t(reg & 0xFF) };
  if (this->write(reg_data, 2) != i2c::ERROR_OK) return false;
  return this->read(&value, 1) == i2c::ERROR_OK;
}

bool Tab5Camera::write_sensor_regs_(const uint16_t regs[][2], size_t count) {
  for (size_t i=0;i<count;i++) {
    if (!this->write_sensor_reg_(regs[i][0], regs[i][1])) return false;
    delay(1);
  }
  return true;
}

CameraResolutionInfo Tab5Camera::get_resolution_info_() {
  switch(this->resolution_) {
    case RESOLUTION_1080P: return {1920,1080};
    case RESOLUTION_720P:  return {1280,720};
    case RESOLUTION_VGA:   return {640,480};
    case RESOLUTION_QVGA:  return {320,240};
    default: return {640,480};
  }
}

bool Tab5Camera::allocate_frame_buffer_() {
  CameraResolutionInfo res_info = this->get_resolution_info_();
  size_t buffer_size = res_info.width * res_info.height * 2;  // RGB565 par défaut

  this->frame_buffer_.buffer = (uint8_t*)malloc(buffer_size);
  if (!this->frame_buffer_.buffer) return false;

  this->frame_buffer_.length = buffer_size;
  this->frame_buffer_.width = res_info.width;
  this->frame_buffer_.height = res_info.height;
  this->frame_buffer_.format = this->pixel_format_;
  return true;
}

void Tab5Camera::free_frame_buffer_() {
  if (this->frame_buffer_.buffer) {
    free(this->frame_buffer_.buffer);
    this->frame_buffer_.buffer = nullptr;
    this->frame_buffer_.length = 0;
  }
}

bool Tab5Camera::init_csi_interface_() {
#ifdef CONFIG_ISP_ENABLED
  if (this->frame_buffer_.width == 0 || this->frame_buffer_.height == 0) return false;

  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = this->frame_buffer_.width;
  csi_config.v_res = this->frame_buffer_.height;
  csi_config.lane_bit_rate_mbps = 576;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 1;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 1;

  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_ctlr_handle_);
  if (ret != ESP_OK) return false;

  esp_cam_ctlr_evt_cbs_t cbs = {};
  cbs.on_get_new_trans = [](esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) -> bool {
    Tab5Camera *camera = (Tab5Camera*)user_data;
    trans->buffer = camera->frame_buffer_.buffer;
    trans->buflen = camera->frame_buffer_.length;
    return true;
  };
  cbs.on_trans_finished = [](esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) -> bool { return true; };

  ret = esp_cam_ctlr_register_event_callbacks(this->cam_ctlr_handle_, &cbs, this);
  if (ret != ESP_OK) return false;

  ret = esp_cam_ctlr_enable(this->cam_ctlr_handle_);
  if (ret != ESP_OK) return false;

  ret = esp_cam_ctlr_start(this->cam_ctlr_handle_);
  if (ret != ESP_OK) return false;

  this->csi_initialized_ = true;
  return true;
#else
  return false;
#endif
}

bool Tab5Camera::capture_csi_frame_() {
#ifdef CONFIG_ISP_ENABLED
  if (!this->cam_ctlr_handle_) return false;

  esp_cam_ctlr_trans_t trans = {};
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_ctlr_handle_, &trans, 100);
  return ret == ESP_OK;
#else
  return false;
#endif
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) return false;
  return this->capture_csi_frame_();
}

bool Tab5Camera::take_snapshot() {
  return this->capture_frame();
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_) return false;
  if (this->streaming_) return true;
  this->streaming_ = true;
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_) return true;
  this->streaming_ = false;
  return true;
}

CameraFrameBuffer* Tab5Camera::get_frame_buffer() {
  if (!this->initialized_) return nullptr;
  return &this->frame_buffer_;
}

void Tab5Camera::return_frame_buffer() {
  // Pour l'instant, rien à libérer explicitement
}

}  // namespace tab5_camera
}  // namespace esphome



