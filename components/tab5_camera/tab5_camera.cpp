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
// Configuration 1080P (1920x1080)
static const uint16_t SC202CS_1080P_REGS[][2] = {
    {0x0103, 0x01}, {0x0100, 0x00}, {0x36e9, 0x80}, {0x37f9, 0x80},
    {0x3200, 0x00}, {0x3201, 0x00}, {0x3202, 0x00}, {0x3203, 0x00},
    {0x3204, 0x07}, {0x3205, 0x8f}, {0x3206, 0x04}, {0x3207, 0x47},
    {0x3208, 0x07}, {0x3209, 0x80}, {0x320a, 0x04}, {0x320b, 0x38},
    {0x320c, 0x04}, {0x320d, 0x4c}, {0x320e, 0x04}, {0x320f, 0x66},
    {0x3301, 0x06}, {0x3304, 0x50}, {0x3306, 0x48}, {0x3308, 0x18},
    {0x3309, 0x68}, {0x330b, 0xe8}, {0x330d, 0x28}, {0x330e, 0x48},
    {0x3314, 0x94}, {0x331e, 0x41}, {0x331f, 0x61}, {0x3333, 0x10},
    {0x3334, 0x40}, {0x335e, 0x06}, {0x335f, 0x0a}, {0x3364, 0x5e},
    {0x337c, 0x02}, {0x337d, 0x0a}, {0x3390, 0x01}, {0x3391, 0x03},
    {0x3392, 0x07}, {0x3393, 0x06}, {0x3394, 0x06}, {0x3395, 0x06},
    {0x3630, 0xf0}, {0x3633, 0x33}, {0x3634, 0x64}, {0x3637, 0x50},
    {0x4501, 0x00}, {0x4509, 0x00}, {0x4837, 0x1e}, {0x0100, 0x01},
};

// Configuration 720P (1280x720)
static const uint16_t SC202CS_720P_REGS[][2] = {
    {0x0103, 0x01}, {0x0100, 0x00}, {0x36e9, 0x80}, {0x37f9, 0x80},
    {0x3200, 0x00}, {0x3201, 0xa0}, {0x3202, 0x00}, {0x3203, 0xf0},
    {0x3204, 0x06}, {0x3205, 0xef}, {0x3206, 0x03}, {0x3207, 0x57},
    {0x3208, 0x05}, {0x3209, 0x00}, {0x320a, 0x02}, {0x320b, 0xd0},
    {0x320c, 0x04}, {0x320d, 0x4c}, {0x320e, 0x03}, {0x320f, 0x00},
    {0x3301, 0x06}, {0x3304, 0x50}, {0x3306, 0x48}, {0x3630, 0xf0},
    {0x3633, 0x33}, {0x3634, 0x64}, {0x4501, 0x00}, {0x4509, 0x00},
    {0x4837, 0x1e}, {0x0100, 0x01},
};

// Configuration VGA (640x480)
static const uint16_t SC202CS_VGA_REGS[][2] = {
    {0x0103, 0x01}, {0x0100, 0x00}, {0x36e9, 0x80}, {0x37f9, 0x80},
    {0x3200, 0x01}, {0x3201, 0x48}, {0x3202, 0x01}, {0x3203, 0x38},
    {0x3204, 0x06}, {0x3205, 0x47}, {0x3206, 0x03}, {0x3207, 0x0f},
    {0x3208, 0x02}, {0x3209, 0x80}, {0x320a, 0x01}, {0x320b, 0xe0},
    {0x320c, 0x04}, {0x320d, 0x4c}, {0x320e, 0x02}, {0x320f, 0x08},
    {0x3301, 0x06}, {0x3304, 0x50}, {0x3630, 0xf0}, {0x4501, 0x00},
    {0x4509, 0x00}, {0x4837, 0x1e}, {0x0100, 0x01},
};

// Configuration QVGA (320x240)
static const uint16_t SC202CS_QVGA_REGS[][2] = {
    {0x0103, 0x01}, {0x0100, 0x00}, {0x36e9, 0x80}, {0x37f9, 0x80},
    {0x3200, 0x01}, {0x3201, 0x48}, {0x3202, 0x01}, {0x3203, 0x38},
    {0x3204, 0x06}, {0x3205, 0x47}, {0x3206, 0x03}, {0x3207, 0x0f},
    {0x3208, 0x01}, {0x3209, 0x40}, {0x320a, 0x00}, {0x320b, 0xf0},
    {0x320c, 0x04}, {0x320d, 0x4c}, {0x320e, 0x01}, {0x320f, 0x04},
    {0x3301, 0x06}, {0x3304, 0x50}, {0x3630, 0xf0}, {0x4501, 0x00},
    {0x4509, 0x00}, {0x4837, 0x1e}, {0x0100, 0x01},
};

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
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "Caméra Tab5 initialisée avec succès");
}

void Tab5Camera::loop() {
  // Loop peut être utilisé pour des tâches périodiques si nécessaire
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
  
  if (this->initialized_) {
    ESP_LOGCONFIG(TAG, "  État: Initialisé");
  } else {
    ESP_LOGCONFIG(TAG, "  État: Non initialisé");
  }
}

bool Tab5Camera::start_external_clock_() {
  if (this->xclk_pin_ == nullptr) {
    ESP_LOGE(TAG, "Pin clock externe non configuré");
    return false;
  }
  
  int gpio_num = -1;
  
  #ifdef USE_ESP32
    auto *esp32_pin = (esphome::esp32::ESP32InternalGPIOPin*)this->xclk_pin_;
    gpio_num = esp32_pin->get_pin();
  #else
    ESP_LOGE(TAG, "Plateforme non ESP32");
    return false;
  #endif
  
  if (gpio_num < 0) {
    ESP_LOGE(TAG, "Numéro de GPIO invalide");
    return false;
  }
  
  ESP_LOGI(TAG, "Initialisation LEDC sur GPIO%d à %u Hz", gpio_num, this->xclk_frequency_);
  
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.freq_hz = this->xclk_frequency_;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer.deconfigure = false;
  
  esp_err_t err = ledc_timer_config(&ledc_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Échec configuration timer LEDC: %d (%s)", err, esp_err_to_name(err));
    ESP_LOGW(TAG, "Tentative avec résolution 2 bits...");
    ledc_timer.duty_resolution = LEDC_TIMER_2_BIT;
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Échec configuration timer LEDC (2 bits): %d (%s)", err, esp_err_to_name(err));
      return false;
    }
  }
  
  ledc_channel_config_t ledc_channel = {};
  ledc_channel.gpio_num = gpio_num;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.duty = (ledc_timer.duty_resolution == LEDC_TIMER_1_BIT) ? 1 : 2;
  ledc_channel.hpoint = 0;
  ledc_channel.flags.output_invert = 0;
  
  err = ledc_channel_config(&ledc_channel);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Échec configuration canal LEDC: %d (%s)", err, esp_err_to_name(err));
    return false;
  }
  
  ESP_LOGI(TAG, "Clock externe démarré à %u Hz sur GPIO%d", this->xclk_frequency_, gpio_num);
  return true;
}

bool Tab5Camera::reset_sensor_() {
  if (this->reset_pin_ == nullptr) {
    return true;
  }
  
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
  
  if (!this->read_sensor_reg_(0x3107, id_high)) {
    ESP_LOGE(TAG, "Impossible de lire l'ID high du capteur");
    return false;
  }
  
  if (!this->read_sensor_reg_(0x3108, id_low)) {
    ESP_LOGE(TAG, "Impossible de lire l'ID low du capteur");
    return false;
  }
  
  uint16_t chip_id = (id_high << 8) | id_low;
  ESP_LOGI(TAG, "Chip ID détecté: 0x%04X", chip_id);
  
  if (chip_id != SC202CS_CHIP_ID_VALUE && chip_id != SC2356_CHIP_ID_VALUE) {
    ESP_LOGW(TAG, "Chip ID inattendu (attendu: 0x%04X ou 0x%04X, reçu: 0x%04X)", 
             SC202CS_CHIP_ID_VALUE, SC2356_CHIP_ID_VALUE, chip_id);
  } else {
    const char* sensor_name = (chip_id == SC202CS_CHIP_ID_VALUE) ? "SC202CS" : "SC2356";
    ESP_LOGI(TAG, "Capteur détecté: %s", sensor_name);
  }
  
  return true;
}

bool Tab5Camera::configure_sc202cs_() {
  if (!this->write_sensor_reg_(SC202CS_RESET_REG, 0x01)) {
    ESP_LOGE(TAG, "Échec du soft reset");
    return false;
  }
  delay(100);
  
  const uint16_t (*regs)[2] = nullptr;
  size_t reg_count = 0;
  
  switch (this->resolution_) {
    case RESOLUTION_1080P:
      regs = SC202CS_1080P_REGS;
      reg_count = sizeof(SC202CS_1080P_REGS) / sizeof(SC202CS_1080P_REGS[0]);
      break;
    case RESOLUTION_720P:
      regs = SC202CS_720P_REGS;
      reg_count = sizeof(SC202CS_720P_REGS) / sizeof(SC202CS_720P_REGS[0]);
      break;
    case RESOLUTION_VGA:
      regs = SC202CS_VGA_REGS;
      reg_count = sizeof(SC202CS_VGA_REGS) / sizeof(SC202CS_VGA_REGS[0]);
      break;
    case RESOLUTION_QVGA:
      regs = SC202CS_QVGA_REGS;
      reg_count = sizeof(SC202CS_QVGA_REGS) / sizeof(SC202CS_QVGA_REGS[0]);
      break;
  }
  
  if (!this->write_sensor_regs_(regs, reg_count)) {
    ESP_LOGE(TAG, "Échec de l'écriture de la configuration");
    return false;
  }
  
  // Vérification critique du registre test pattern
  uint8_t test_pattern_check = 0xFF;
  if (this->read_sensor_reg_(0x4501, test_pattern_check)) {
    ESP_LOGI(TAG, "Registre 0x4501 (test pattern) = 0x%02X", test_pattern_check);
    if (test_pattern_check != 0x00) {
      ESP_LOGW(TAG, "Test pattern encore actif (0x%02X) ! Forçage à 0x00...", test_pattern_check);
      this->write_sensor_reg_(0x4501, 0x00);
      delay(10);
      
      // Re-vérification
      if (this->read_sensor_reg_(0x4501, test_pattern_check)) {
        ESP_LOGI(TAG, "Après correction: 0x4501 = 0x%02X", test_pattern_check);
      }
    } else {
      ESP_LOGI(TAG, "Test pattern correctement désactivé");
    }
  }
  
  // Vérification du streaming
  uint8_t streaming_check = 0xFF;
  if (this->read_sensor_reg_(0x0100, streaming_check)) {
    ESP_LOGI(TAG, "Registre 0x0100 (streaming) = 0x%02X", streaming_check);
  }
  
  ESP_LOGI(TAG, "Capteur SC202CS configuré avec succès");
  return true;
}

bool Tab5Camera::write_sensor_reg_(uint16_t reg, uint8_t value) {
  uint8_t data[3] = {
      static_cast<uint8_t>((reg >> 8) & 0xFF),
      static_cast<uint8_t>(reg & 0xFF),
      value
  };
  
  return this->write(data, 3) == i2c::ERROR_OK;
}

bool Tab5Camera::read_sensor_reg_(uint16_t reg, uint8_t &value) {
  uint8_t reg_data[2] = {
      static_cast<uint8_t>((reg >> 8) & 0xFF),
      static_cast<uint8_t>(reg & 0xFF)
  };
  
  if (this->write(reg_data, 2) != i2c::ERROR_OK) {
    return false;
  }
  
  return this->read(&value, 1) == i2c::ERROR_OK;
}

bool Tab5Camera::write_sensor_regs_(const uint16_t regs[][2], size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (!this->write_sensor_reg_(regs[i][0], regs[i][1])) {
      ESP_LOGE(TAG, "Échec écriture registre 0x%04X", regs[i][0]);
      return false;
    }
    delay(1);
  }
  return true;
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

bool Tab5Camera::allocate_frame_buffer_() {
  CameraResolutionInfo res_info = this->get_resolution_info_();
  size_t buffer_size = 0;
  
  switch (this->pixel_format_) {
    case PIXEL_FORMAT_RGB565:
    case PIXEL_FORMAT_YUV422:
      buffer_size = res_info.width * res_info.height * 2;
      break;
    case PIXEL_FORMAT_RAW8:
      buffer_size = res_info.width * res_info.height;
      break;
    case PIXEL_FORMAT_JPEG:
      buffer_size = res_info.width * res_info.height / 2;
      break;
  }
  
  this->frame_buffer_.buffer = (uint8_t *)malloc(buffer_size);
  if (this->frame_buffer_.buffer == nullptr) {
    ESP_LOGE(TAG, "Échec allocation buffer (%u bytes)", buffer_size);
    return false;
  }
  
  this->frame_buffer_.length = buffer_size;
  this->frame_buffer_.width = res_info.width;
  this->frame_buffer_.height = res_info.height;
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "Buffer alloué: %u bytes", buffer_size);
  return true;
}

void Tab5Camera::free_frame_buffer_() {
  if (this->frame_buffer_.buffer != nullptr) {
    free(this->frame_buffer_.buffer);
    this->frame_buffer_.buffer = nullptr;
    this->frame_buffer_.length = 0;
  }
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Caméra non initialisée");
    return false;
  }
  
  if (!this->csi_initialized_) {
    if (!this->init_csi_interface_()) {
      ESP_LOGV(TAG, "CSI non disponible, utilisation pattern de test");
      return this->generate_test_pattern_();
    }
  }
  
  if (this->capture_csi_frame_()) {
    return true;
  }
  
  return this->generate_test_pattern_();
}

bool Tab5Camera::generate_test_pattern_() {
  uint16_t *pixels = (uint16_t*)this->frame_buffer_.buffer;
  static uint8_t frame_counter = 0;
  frame_counter++;
  
  for (size_t y = 0; y < this->frame_buffer_.height; y++) {
    for (size_t x = 0; x < this->frame_buffer_.width; x++) {
      size_t idx = y * this->frame_buffer_.width + x;
      
      uint8_t r = (x * 255 / this->frame_buffer_.width) & 0x1F;
      uint8_t g = ((y + frame_counter) * 255 / this->frame_buffer_.height) & 0x3F;
      uint8_t b = ((x + y + frame_counter) * 255 / (this->frame_buffer_.width + this->frame_buffer_.height)) & 0x1F;
      
      pixels[idx] = (r << 11) | (g << 5) | b;
    }
  }
  
  ESP_LOGV(TAG, "Pattern test généré (frame %d)", frame_counter);
  return true;
}

bool Tab5Camera::init_csi_interface_() {
  ESP_LOGI(TAG, "Initialisation interface CSI ESP32-P4 (config M5Stack)...");
  
  #ifdef CONFIG_ISP_ENABLED
  
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
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec création contrôleur CSI: %s", esp_err_to_name(ret));
    return false;
  }
  
  esp_cam_ctlr_evt_cbs_t cbs = {};
  cbs.on_get_new_trans = [](esp_cam_ctlr_handle_t handle, 
                             esp_cam_ctlr_trans_t *trans, 
                             void *user_data) -> bool {
    Tab5Camera *camera = (Tab5Camera*)user_data;
    trans->buffer = camera->frame_buffer_.buffer;
    trans->buflen = camera->frame_buffer_.length;
    return true;
  };
  
  cbs.on_trans_finished = [](esp_cam_ctlr_handle_t handle, 
                              esp_cam_ctlr_trans_t *trans, 
                              void *user_data) -> bool {
    ESP_LOGV("csi", "Frame CSI reçue: %u bytes", trans->received_size);
    return true;
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->cam_ctlr_handle_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec enregistrement callbacks: %s", esp_err_to_name(ret));
    esp_cam_ctlr_del(this->cam_ctlr_handle_);
    this->cam_ctlr_handle_ = nullptr;
    return false;
  }
  
  ret = esp_cam_ctlr_enable(this->cam_ctlr_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec activation contrôleur: %s", esp_err_to_name(ret));
    esp_cam_ctlr_del(this->cam_ctlr_handle_);
    this->cam_ctlr_handle_ = nullptr;
    return false;
  }
  
  ret = esp_cam_ctlr_start(this->cam_ctlr_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec démarrage CSI: %s", esp_err_to_name(ret));
    esp_cam_ctlr_disable(this->cam_ctlr_handle_);
    esp_cam_ctlr_del(this->cam_ctlr_handle_);
    this->cam_ctlr_handle_ = nullptr;
    return false;
  }
  
  this->csi_initialized_ = true;
  ESP_LOGI(TAG, "Interface CSI initialisée M5Stack: %ux%u, 1 lane @ 576 Mbps, RAW8→RGB565", 
           this->frame_buffer_.width, this->frame_buffer_.height);
  return true;
  
  #else
  ESP_LOGW(TAG, "ISP non activé dans sdkconfig");
  return false;
  #endif
}

bool Tab5Camera::capture_csi_frame_() {
  #ifdef CONFIG_ISP_ENABLED
  
  if (this->cam_ctlr_handle_ == nullptr) {
    return false;
  }
  
  esp_cam_ctlr_trans_t trans = {};
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_ctlr_handle_, &trans, 100);
  
  if (ret == ESP_OK) {
    return true;
  } else if (ret == ESP_ERR_TIMEOUT) {
    ESP_LOGV(TAG, "Timeout CSI");
    return false;
  } else {
    ESP_LOGW(TAG, "Erreur CSI: %s", esp_err_to_name(ret));
    return false;
  }
  
  #else
  return false;
  #endif
}

bool Tab5Camera::take_snapshot() {
  ESP_LOGI(TAG, "Prise d'un snapshot...");
  return this->capture_frame();
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Caméra non initialisée");
    return false;
  }
  
  if (this->streaming_) {
    ESP_LOGW(TAG, "Streaming déjà actif");
    return true;
  }
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "Streaming démarré");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_) {
    ESP_LOGW(TAG, "Streaming déjà arrêté");
    return true;
  }
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "Streaming arrêté");
  return true;
}

CameraFrameBuffer *Tab5Camera::get_frame_buffer() {
  if (!this->initialized_) {
    return nullptr;
  }
  return &this->frame_buffer_;
}

void Tab5Camera::return_frame_buffer() {
  // Libérer les ressources si nécessaire
}

}  // namespace tab5_camera
}  // namespace esphome





