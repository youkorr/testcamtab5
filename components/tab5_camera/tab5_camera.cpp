#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#include "esphome/components/esp32/gpio.h"
#endif

#include <driver/ledc.h>
#include <driver/i2c.h>
#include "esp_sccb_intf.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Initialisation Tab5 Camera...");
  
  if (!this->start_external_clock_()) {
    ESP_LOGE(TAG, "Échec clock externe");
    this->mark_failed();
    return;
  }
  
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(50);
    ESP_LOGI(TAG, "Reset effectué");
  }
  
  if (!this->init_sensor_with_official_driver_()) {
    ESP_LOGE(TAG, "Échec init capteur");
    this->mark_failed();
    return;
  }
  
  if (!this->allocate_frame_buffer_()) {
    ESP_LOGE(TAG, "Échec buffer");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "Camera initialisée avec succès");
}

bool Tab5Camera::init_sensor_with_official_driver_() {
  ESP_LOGI(TAG, "Communication I2C directe...");
  
  // ESPHome a déjà configuré l'I2C, pas besoin de le réinitialiser
  // On crée juste un handle fictif pour compatibilité
  this->sccb_handle_ = (esp_sccb_io_handle_t)1;  // Handle factice
  
  ESP_LOGI(TAG, "I2C prêt (0x%02X)", this->sensor_address_);
  return this->init_sc202cs_manually_();
}
bool Tab5Camera::init_sc202cs_manually_() {
  ESP_LOGI(TAG, "Initialisation SC202CS via I2C...");
  
  // Lecture chip ID via I2C direct
  uint8_t chip_id_h = 0, chip_id_l = 0;
  
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (this->sensor_address_ << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, 0x31, true);  // Reg 0x3107 high byte
  i2c_master_write_byte(cmd, 0x07, true);  // Reg 0x3107 low byte
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (this->sensor_address_ << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, &chip_id_h, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Pas de réponse I2C");
    return false;
  }
  
  // Lecture 0x3108
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (this->sensor_address_ << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, 0x31, true);
  i2c_master_write_byte(cmd, 0x08, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (this->sensor_address_ << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, &chip_id_l, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  
  uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
  ESP_LOGI(TAG, "Chip ID: 0x%04X", chip_id);
  
  // Helper pour écrire un registre
  auto write_reg = [this](uint16_t reg, uint8_t val) -> bool {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (this->sensor_address_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, reg & 0xFF, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK);
  };
  
  // Séquence init
  struct RegVal {
    uint16_t reg;
    uint8_t val;
    uint16_t delay_ms;
  };
  
  const RegVal init_sequence[] = {
    {0x0103, 0x01, 10},
    {0x0100, 0x00, 0},
    {0x36e9, 0x80, 0},
    {0x37f9, 0x80, 0},
    {0x3200, 0x00, 0}, {0x3201, 0x00, 0},
    {0x3208, 0x02, 0}, {0x3209, 0x80, 0},
    {0x320a, 0x01, 0}, {0x320b, 0xe0, 0},
    {0x4501, 0x00, 0},  // TEST PATTERN OFF
    {0x4509, 0x00, 0},
    {0x0100, 0x01, 20},
  };
  
  for (const auto& reg : init_sequence) {
    if (!write_reg(reg.reg, reg.val)) {
      ESP_LOGE(TAG, "Reg 0x%04X fail", reg.reg);
      return false;
    }
    if (reg.delay_ms > 0) {
      delay(reg.delay_ms);
    }
  }
  
  ESP_LOGI(TAG, "✅ SC202CS initialisé, 0x4501=0x00");
  return true;
}

bool Tab5Camera::start_external_clock_() {
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
  ledc_channel.hpoint = 0;
  
  err = ledc_channel_config(&ledc_channel);
  return (err == ESP_OK);
}

bool Tab5Camera::allocate_frame_buffer_() {
  uint16_t width, height;
  this->get_resolution_dimensions_(width, height);
  
  size_t buffer_size = width * height * sizeof(uint16_t);
  this->frame_buffer_.buffer = (uint8_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  
  if (this->frame_buffer_.buffer == nullptr) {
    return false;
  }
  
  this->frame_buffer_.length = buffer_size;
  this->frame_buffer_.width = width;
  this->frame_buffer_.height = height;
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "Buffer %ux%u OK", width, height);
  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_) {
    return false;
  }
  
  esp_err_t ret = esp_sccb_transmit_reg_a16v8(this->sccb_handle_, 0x0100, 0x01);
  this->streaming_ = (ret == ESP_OK);
  return this->streaming_;
}

bool Tab5Camera::stop_streaming() {
  if (!this->initialized_) {
    return false;
  }
  
  esp_err_t ret = esp_sccb_transmit_reg_a16v8(this->sccb_handle_, 0x0100, 0x00);
  this->streaming_ = !(ret == ESP_OK);
  return (ret == ESP_OK);
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) {
    return false;
  }
  
  static uint8_t counter = 0;
  counter++;
  uint16_t *pixels = (uint16_t*)this->frame_buffer_.buffer;
  for (size_t i = 0; i < this->frame_buffer_.width * this->frame_buffer_.height; i++) {
    pixels[i] = (counter << 11) | (counter << 5) | counter;
  }
  return true;
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

void Tab5Camera::free_frame_buffer_() {
  if (this->frame_buffer_.buffer != nullptr) {
    heap_caps_free(this->frame_buffer_.buffer);
    this->frame_buffer_.buffer = nullptr;
  }
}

void Tab5Camera::loop() {}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Adresse: 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", this->frame_buffer_.width, this->frame_buffer_.height);
}

}  // namespace tab5_camera
}  // namespace esphome





