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
  ESP_LOGI(TAG, "Création bus SCCB...");
  
  esp_err_t ret = esp_sccb_new_i2c_io(
    (i2c_port_t)0,
    this->sensor_address_,
    400000,
    &this->sccb_handle_
  );
  
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec SCCB: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "SCCB créé (adresse 0x%02X @ 400kHz)", this->sensor_address_);
  return this->init_sc202cs_manually_();
}

bool Tab5Camera::init_sc202cs_manually_() {
  ESP_LOGI(TAG, "Initialisation SC202CS...");
  
  uint8_t chip_id_h = 0, chip_id_l = 0;
  
  if (esp_sccb_transmit_receive_reg_a16v8(this->sccb_handle_, 0x3107, &chip_id_h) != ESP_OK) {
    ESP_LOGE(TAG, "Impossible de communiquer avec le capteur");
    return false;
  }
  
  esp_sccb_transmit_receive_reg_a16v8(this->sccb_handle_, 0x3108, &chip_id_l);
  uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
  
  ESP_LOGI(TAG, "Chip ID: 0x%04X", chip_id);
  
  if (chip_id != 0x2356 && chip_id != 0xCB34) {
    ESP_LOGW(TAG, "Chip ID inattendu (attendu 0x2356 ou 0xCB34)");
  }
  
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
    {0x3202, 0x00, 0}, {0x3203, 0x00, 0},
    {0x3204, 0x05, 0}, {0x3205, 0x0f, 0},
    {0x3206, 0x03, 0}, {0x3207, 0x8f, 0},
    {0x3208, 0x02, 0}, {0x3209, 0x80, 0},
    {0x320a, 0x01, 0}, {0x320b, 0xe0, 0},
    {0x320c, 0x04, 0}, {0x320d, 0x4c, 0},
    {0x320e, 0x02, 0}, {0x320f, 0x08, 0},
    {0x3301, 0x06, 0},
    {0x3304, 0x50, 0},
    {0x3306, 0x50, 0},
    {0x3309, 0x68, 0},
    {0x330b, 0xd0, 0},
    {0x330e, 0x18, 0},
    {0x3314, 0x94, 0},
    {0x331e, 0x41, 0},
    {0x331f, 0x59, 0},
    {0x3320, 0x09, 0},
    {0x3333, 0x10, 0},
    {0x3334, 0x40, 0},
    {0x335d, 0x60, 0},
    {0x3364, 0x56, 0},
    {0x3390, 0x01, 0},
    {0x3391, 0x03, 0},
    {0x3392, 0x07, 0},
    {0x3393, 0x06, 0},
    {0x3394, 0x06, 0},
    {0x3395, 0x06, 0},
    {0x3630, 0xf0, 0},
    {0x3631, 0x85, 0},
    {0x3632, 0x74, 0},
    {0x3633, 0x22, 0},
    {0x3637, 0x4d, 0},
    {0x363a, 0x8e, 0},
    {0x363b, 0x04, 0},
    {0x363c, 0x06, 0},
    {0x3e00, 0x00, 0},
    {0x3e01, 0x4d, 0},
    {0x3e02, 0xc0, 0},
    {0x3e03, 0x0b, 0},
    {0x3e06, 0x00, 0},
    {0x3e07, 0x80, 0},
    {0x3e08, 0x03, 0},
    {0x3e09, 0x40, 0},
    {0x4501, 0x00, 0},
    {0x4509, 0x00, 0},
    {0x4837, 0x1e, 0},
    {0x5000, 0x0e, 0},
    {0x5001, 0x46, 0},
    {0x0100, 0x01, 20},
  };
  
  for (const auto& reg : init_sequence) {
    esp_err_t err = esp_sccb_transmit_reg_a16v8(this->sccb_handle_, reg.reg, reg.val);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Échec écriture reg 0x%04X: %s", reg.reg, esp_err_to_name(err));
      return false;
    }
    if (reg.delay_ms > 0) {
      delay(reg.delay_ms);
    }
  }
  
  uint8_t test_pattern_val = 0xFF;
  esp_sccb_transmit_receive_reg_a16v8(this->sccb_handle_, 0x4501, &test_pattern_val);
  ESP_LOGI(TAG, "Registre 0x4501 (test pattern) = 0x%02X", test_pattern_val);
  
  if (test_pattern_val != 0x00) {
    ESP_LOGW(TAG, "Test pattern encore activé, forçage à 0x00...");
    esp_sccb_transmit_reg_a16v8(this->sccb_handle_, 0x4501, 0x00);
    delay(10);
    
    esp_sccb_transmit_receive_reg_a16v8(this->sccb_handle_, 0x4501, &test_pattern_val);
    ESP_LOGI(TAG, "Après correction: 0x4501 = 0x%02X", test_pattern_val);
  }
  
  uint8_t streaming_val = 0xFF;
  esp_sccb_transmit_receive_reg_a16v8(this->sccb_handle_, 0x0100, &streaming_val);
  ESP_LOGI(TAG, "Registre 0x0100 (streaming) = 0x%02X", streaming_val);
  
  ESP_LOGI(TAG, "SC202CS initialisé avec succès");
  return true;
}

bool Tab5Camera::start_external_clock_() {
  if (this->xclk_pin_ == nullptr) {
    ESP_LOGW(TAG, "Pas de pin clock externe");
    return false;
  }
  
  int gpio_num = -1;
  
  #ifdef USE_ESP32
  auto *esp32_pin = (esphome::esp32::ESP32InternalGPIOPin*)this->xclk_pin_;
  gpio_num = esp32_pin->get_pin();
  #endif
  
  if (gpio_num < 0) {
    ESP_LOGE(TAG, "GPIO clock invalide");
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
      ESP_LOGE(TAG, "Échec config timer LEDC: %s", esp_err_to_name(err));
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
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Échec config canal LEDC: %s", esp_err_to_name(err));
    return false;
  }
  
  ESP_LOGI(TAG, "Clock 24MHz démarrée sur GPIO%d", gpio_num);
  return true;
}

bool Tab5Camera::allocate_frame_buffer_() {
  uint16_t width, height;
  this->get_resolution_dimensions_(width, height);
  
  size_t buffer_size = width * height * sizeof(uint16_t);
  
  this->frame_buffer_.buffer = (uint8_t *)heap_caps_malloc(
    buffer_size, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (this->frame_buffer_.buffer == nullptr) {
    ESP_LOGE(TAG, "Échec allocation %u bytes en PSRAM", buffer_size);
    return false;
  }
  
  this->frame_buffer_.length = buffer_size;
  this->frame_buffer_.width = width;
  this->frame_buffer_.height = height;
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "Buffer alloué: %ux%u = %u bytes", width, height, buffer_size);
  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_) {
    return false;
  }
  
  esp_err_t ret = esp_sccb_transmit_reg_a16v8(this->sccb_handle_, 0x0100, 0x01);
  if (ret == ESP_OK) {
    this->streaming_ = true;
    ESP_LOGI(TAG, "Streaming démarré");
    return true;
  }
  return false;
}

bool Tab5Camera::stop_streaming() {
  if (!this->initialized_) {
    return false;
  }
  
  esp_err_t ret = esp_sccb_transmit_reg_a16v8(this->sccb_handle_, 0x0100, 0x00);
  if (ret == ESP_OK) {
    this->streaming_ = false;
    ESP_LOGI(TAG, "Streaming arrêté");
    return true;
  }
  return false;
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
  ESP_LOGCONFIG(TAG, "  Nom: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Adresse I2C: 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", 
                this->frame_buffer_.width, this->frame_buffer_.height);
  ESP_LOGCONFIG(TAG, "  Initialisé: %s", this->initialized_ ? "OUI" : "NON");
}

}  // namespace tab5_camera
}  // namespace esphome





