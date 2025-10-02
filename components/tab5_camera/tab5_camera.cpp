#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#include "esphome/components/esp32/gpio.h"
#endif

#include <driver/ledc.h>
#include <driver/i2c.h>

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

// Helpers I2C
static bool i2c_write_reg16(uint8_t addr, uint16_t reg, uint8_t val) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
  i2c_master_write_byte(cmd, reg & 0xFF, true);
  i2c_master_write_byte(cmd, val, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return (ret == ESP_OK);
}

static bool i2c_read_reg16(uint8_t addr, uint16_t reg, uint8_t *val) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
  i2c_master_write_byte(cmd, reg & 0xFF, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return (ret == ESP_OK);
}

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Initialisation Tab5 Camera...");
  
  if (!this->start_external_clock_()) {
    ESP_LOGE(TAG, "Échec clock");
    this->mark_failed();
    return;
  }
  
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(50);
    ESP_LOGI(TAG, "Reset OK");
  }
  
  if (!this->init_sensor_with_official_driver_()) {
    ESP_LOGE(TAG, "Échec capteur");
    this->mark_failed();
    return;
  }
  
  if (!this->allocate_frame_buffer_()) {
    ESP_LOGE(TAG, "Échec buffer");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "Camera OK");
}

bool Tab5Camera::init_sensor_with_official_driver_() {
  ESP_LOGI(TAG, "Config I2C...");
  
  i2c_config_t i2c_conf = {};
  i2c_conf.mode = I2C_MODE_MASTER;
  i2c_conf.sda_io_num = (gpio_num_t)31;
  i2c_conf.scl_io_num = (gpio_num_t)32;
  i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_conf.master.clk_speed = 400000;
  
  esp_err_t ret = i2c_param_config(I2C_NUM_0, &i2c_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C config fail");
    return false;
  }
  
  ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "I2C install fail");
    return false;
  }
  
  this->sccb_handle_ = (esp_sccb_io_handle_t)1;
  ESP_LOGI(TAG, "I2C OK");
  return this->init_sc202cs_manually_();
}

bool Tab5Camera::init_sc202cs_manually_() {
  ESP_LOGI(TAG, "Init SC202CS...");
  
  uint8_t chip_id_h = 0, chip_id_l = 0;
  
  if (!i2c_read_reg16(this->sensor_address_, 0x3107, &chip_id_h)) {
    ESP_LOGE(TAG, "Pas de I2C");
    return false;
  }
  
  i2c_read_reg16(this->sensor_address_, 0x3108, &chip_id_l);
  uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
  
  ESP_LOGI(TAG, "Chip ID: 0x%04X", chip_id);
  
  struct RegVal {
    uint16_t reg;
    uint8_t val;
    uint16_t delay_ms;
  };
  
  const RegVal seq[] = {
    {0x0103, 0x01, 10},
    {0x0100, 0x00, 0},
    {0x36e9, 0x80, 0},
    {0x37f9, 0x80, 0},
    {0x3208, 0x02, 0}, {0x3209, 0x80, 0},
    {0x320a, 0x01, 0}, {0x320b, 0xe0, 0},
    {0x4501, 0x00, 0},
    {0x4509, 0x00, 0},
    {0x0100, 0x01, 20},
  };
  
  for (const auto& r : seq) {
    if (!i2c_write_reg16(this->sensor_address_, r.reg, r.val)) {
      ESP_LOGE(TAG, "Reg 0x%04X fail", r.reg);
      return false;
    }
    if (r.delay_ms > 0) delay(r.delay_ms);
  }
  
  uint8_t test = 0xFF;
  i2c_read_reg16(this->sensor_address_, 0x4501, &test);
  ESP_LOGI(TAG, "0x4501 = 0x%02X", test);
  
  return true;
}

bool Tab5Camera::start_external_clock_() {
  if (this->xclk_pin_ == nullptr) return false;
  
  int gpio_num = -1;
  #ifdef USE_ESP32
  auto *esp32_pin = (esphome::esp32::ESP32InternalGPIOPin*)this->xclk_pin_;
  gpio_num = esp32_pin->get_pin();
  #endif
  
  if (gpio_num < 0) return false;
  
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
    if (err != ESP_OK) return false;
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
  
  if (this->frame_buffer_.buffer == nullptr) return false;
  
  this->frame_buffer_.length = buffer_size;
  this->frame_buffer_.width = width;
  this->frame_buffer_.height = height;
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "Buffer %ux%u OK", width, height);
  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_) return false;
  
  bool ok = i2c_write_reg16(this->sensor_address_, 0x0100, 0x01);
  this->streaming_ = ok;
  return ok;
}

bool Tab5Camera::stop_streaming() {
  if (!this->initialized_) return false;
  
  bool ok = i2c_write_reg16(this->sensor_address_, 0x0100, 0x00);
  if (ok) this->streaming_ = false;
  return ok;
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) return false;
  
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





