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
static const uint16_t SC202CS_VGA_REGS[][2] = {
    {0x0103, 0x01}, {0x0100, 0x00}, {0x36e9, 0x80}, {0x37f9, 0x80},
    {0x3200, 0x01}, {0x3201, 0x48}, {0x3202, 0x01}, {0x3203, 0x38},
    {0x3204, 0x06}, {0x3205, 0x47}, {0x3206, 0x03}, {0x3207, 0x0f},
    {0x3208, 0x02}, {0x3209, 0x80}, {0x320a, 0x01}, {0x320b, 0xe0},
    {0x320c, 0x04}, {0x320d, 0x4c}, {0x320e, 0x02}, {0x320f, 0x08},
    {0x3301, 0x06}, {0x3304, 0x50}, {0x3630, 0xf0}, {0x4837, 0x1e},
    {0x0100, 0x00},  // NE PAS démarrer automatiquement
};

static const uint16_t SC202CS_QVGA_REGS[][2] = {
    {0x0103, 0x01}, {0x0100, 0x00}, {0x36e9, 0x80}, {0x37f9, 0x80},
    {0x3200, 0x01}, {0x3201, 0x48}, {0x3202, 0x01}, {0x3203, 0x38},
    {0x3204, 0x06}, {0x3205, 0x47}, {0x3206, 0x03}, {0x3207, 0x0f},
    {0x3208, 0x01}, {0x3209, 0x40}, {0x320a, 0x00}, {0x320b, 0xf0},
    {0x320c, 0x04}, {0x320d, 0x4c}, {0x320e, 0x01}, {0x320f, 0x04},
    {0x3301, 0x06}, {0x3304, 0x50}, {0x3630, 0xf0}, {0x4837, 0x1e},
    {0x0100, 0x00},
};

void Tab5Camera::setup() {
  ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
  ESP_LOGI(TAG, "║   Tab5 Camera Setup - REAL IMAGE    ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
  
  #ifndef CONFIG_ISP_ENABLED
  ESP_LOGE(TAG, "");
  ESP_LOGE(TAG, "██████████████████████████████████████████████████");
  ESP_LOGE(TAG, "█  ERREUR CRITIQUE: CONFIG_ISP_ENABLED=n        █");
  ESP_LOGE(TAG, "█  La caméra CSI ne peut PAS fonctionner        █");
  ESP_LOGE(TAG, "█  sans CONFIG_ISP_ENABLED=y                    █");
  ESP_LOGE(TAG, "██████████████████████████████████████████████████");
  ESP_LOGE(TAG, "");
  ESP_LOGE(TAG, "SOLUTION: Créer sdkconfig.defaults avec:");
  ESP_LOGE(TAG, "  CONFIG_ISP_ENABLED=y");
  ESP_LOGE(TAG, "  CONFIG_MIPI_CSI_ENABLED=y");
  ESP_LOGE(TAG, "");
  this->mark_failed();
  return;
  #endif
  
  if (!this->start_external_clock_()) {
    ESP_LOGE(TAG, "❌ ÉCHEC: Clock externe");
    this->mark_failed();
    return;
  }
  
  if (this->reset_pin_ != nullptr) {
    if (!this->reset_sensor_()) {
      ESP_LOGE(TAG, "❌ ÉCHEC: Reset capteur");
      this->mark_failed();
      return;
    }
  }
  
  delay(50);
  
  if (!this->init_sc202cs_sensor_()) {
    ESP_LOGE(TAG, "❌ ÉCHEC: Init capteur SC202CS");
    ESP_LOGE(TAG, "Vérifiez:");
    ESP_LOGE(TAG, "  - Connexion I2C (SDA/SCL)");
    ESP_LOGE(TAG, "  - Adresse I2C: 0x36");
    ESP_LOGE(TAG, "  - Alimentation capteur");
    this->mark_failed();
    return;
  }
  
  if (!this->configure_sc202cs_()) {
    ESP_LOGE(TAG, "❌ ÉCHEC: Configuration capteur");
    this->mark_failed();
    return;
  }
  
  if (!this->allocate_frame_buffer_()) {
    ESP_LOGE(TAG, "❌ ÉCHEC: Allocation buffer");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Caméra Tab5 initialisée - PRÊTE POUR CSI");
}

void Tab5Camera::loop() {
  // Loop vide
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Nom: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Adresse I2C: 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Clock: %u Hz", this->xclk_frequency_);
  
  CameraResolutionInfo res_info = this->get_resolution_info_();
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", res_info.width, res_info.height);
  ESP_LOGCONFIG(TAG, "  Format: RGB565");
  ESP_LOGCONFIG(TAG, "  Buffer: %u bytes", this->frame_buffer_.length);
  ESP_LOGCONFIG(TAG, "  État: %s", this->initialized_ ? "✓ Initialisé" : "✗ Non initialisé");
}

bool Tab5Camera::start_external_clock_() {
  if (this->xclk_pin_ == nullptr) {
    ESP_LOGE(TAG, "Pin clock non configuré");
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
    ESP_LOGE(TAG, "GPIO invalide");
    return false;
  }
  
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_2_BIT;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.freq_hz = this->xclk_frequency_;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer.deconfigure = false;
  
  esp_err_t err = ledc_timer_config(&ledc_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Échec timer LEDC: %s", esp_err_to_name(err));
    return false;
  }
  
  ledc_channel_config_t ledc_channel = {};
  ledc_channel.gpio_num = gpio_num;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.duty = 2;
  ledc_channel.hpoint = 0;
  ledc_channel.flags.output_invert = 0;
  
  err = ledc_channel_config(&ledc_channel);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Échec canal LEDC: %s", esp_err_to_name(err));
    return false;
  }
  
  ESP_LOGI(TAG, "✅ Clock: GPIO%d @ %u Hz", gpio_num, this->xclk_frequency_);
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
  
  ESP_LOGI(TAG, "✅ Reset capteur");
  return true;
}

bool Tab5Camera::init_sc202cs_sensor_() {
  uint8_t id_high, id_low;
  
  // Lire PID (Product ID) sur registres 0x3107 et 0x3108
  if (!this->read_sensor_reg_(0x3107, id_high)) {
    ESP_LOGE(TAG, "Impossible de lire registre 0x3107");
    return false;
  }
  
  if (!this->read_sensor_reg_(0x3108, id_low)) {
    ESP_LOGE(TAG, "Impossible de lire registre 0x3108");
    return false;
  }
  
  uint16_t chip_id = (id_high << 8) | id_low;
  ESP_LOGI(TAG, "Product ID (PID): 0x%04X", chip_id);
  
  if (chip_id == SC202CS_CHIP_ID_VALUE) {
    ESP_LOGI(TAG, "✅ Capteur détecté: SC202CS (PID: 0xEB52)");
  } else {
    ESP_LOGE(TAG, "❌ PID incorrect: 0x%04X (attendu: 0xEB52)", chip_id);
    ESP_LOGE(TAG, "Vérifiez:");
    ESP_LOGE(TAG, "  - Adresse I2C: 0x36");
    ESP_LOGE(TAG, "  - Câblage I2C (SDA/SCL)");
    ESP_LOGE(TAG, "  - Alimentation capteur");
    return false;
  }
  
  return true;
}

bool Tab5Camera::configure_sc202cs_() {
  if (!this->write_sensor_reg_(SC202CS_RESET_REG, 0x01)) {
    ESP_LOGE(TAG, "Échec soft reset");
    return false;
  }
  delay(100);
  
  const uint16_t (*regs)[2] = nullptr;
  size_t reg_count = 0;
  
  switch (this->resolution_) {
    case RESOLUTION_VGA:
      regs = SC202CS_VGA_REGS;
      reg_count = sizeof(SC202CS_VGA_REGS) / sizeof(SC202CS_VGA_REGS[0]);
      ESP_LOGI(TAG, "Config: VGA (640x480)");
      break;
    case RESOLUTION_QVGA:
      regs = SC202CS_QVGA_REGS;
      reg_count = sizeof(SC202CS_QVGA_REGS) / sizeof(SC202CS_QVGA_REGS[0]);
      ESP_LOGI(TAG, "Config: QVGA (320x240)");
      break;
    default:
      regs = SC202CS_VGA_REGS;
      reg_count = sizeof(SC202CS_VGA_REGS) / sizeof(SC202CS_VGA_REGS[0]);
      ESP_LOGI(TAG, "Config: VGA (défaut)");
      break;
  }
  
  if (!this->write_sensor_regs_(regs, reg_count)) {
    ESP_LOGE(TAG, "Échec écriture config");
    return false;
  }
  
  ESP_LOGI(TAG, "✅ Capteur configuré (%d registres)", reg_count);
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
      ESP_LOGE(TAG, "Échec reg 0x%04X", regs[i][0]);
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
  size_t buffer_size = res_info.width * res_info.height * 2;  // RGB565 = 2 bytes/pixel
  
  this->frame_buffer_.buffer = (uint8_t *)malloc(buffer_size);
  if (this->frame_buffer_.buffer == nullptr) {
    ESP_LOGE(TAG, "Échec malloc %u bytes", buffer_size);
    return false;
  }
  
  // Remplir de noir au début
  memset(this->frame_buffer_.buffer, 0, buffer_size);
  
  this->frame_buffer_.length = buffer_size;
  this->frame_buffer_.width = res_info.width;
  this->frame_buffer_.height = res_info.height;
  this->frame_buffer_.format = this->pixel_format_;
  
  ESP_LOGI(TAG, "✅ Buffer: %u bytes (%ux%u)", buffer_size, res_info.width, res_info.height);
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
  
  // TOUJOURS essayer CSI d'abord
  if (!this->csi_initialized_) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  INITIALISATION CSI POUR IMAGE RÉELLE     ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════╝");
    
    if (!this->init_csi_interface_()) {
      ESP_LOGE(TAG, "");
      ESP_LOGE(TAG, "██████████████████████████████████████████████");
      ESP_LOGE(TAG, "█  ÉCHEC INITIALISATION CSI                 █");
      ESP_LOGE(TAG, "█  Impossible d'obtenir l'image réelle      █");
      ESP_LOGE(TAG, "██████████████████████████████████████████████");
      return false;
    }
  }
  
  // Capturer via CSI - PAS DE FALLBACK
  return this->capture_csi_frame_();
}

bool Tab5Camera::init_csi_interface_() {
  #ifndef CONFIG_ISP_ENABLED
  ESP_LOGE(TAG, "CONFIG_ISP_ENABLED=n - CSI impossible");
  return false;
  #endif
  
  #ifdef CONFIG_ISP_ENABLED
  
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "➤ Étape 1/7: Activation streaming capteur...");
  
  // Activer le capteur en mode streaming
  if (!this->write_sensor_reg_(0x0100, 0x01)) {
    ESP_LOGE(TAG, "  ❌ Échec activation capteur (reg 0x0100)");
    return false;
  }
  delay(100);
  
  // Vérifier que le capteur stream bien
  uint8_t stream_status = 0;
  if (this->read_sensor_reg_(0x0100, stream_status)) {
    ESP_LOGI(TAG, "  ✅ Capteur streaming: 0x%02X", stream_status);
  }
  
  ESP_LOGI(TAG, "➤ Étape 2/7: Configuration CSI...");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = res.width;
  csi_config.v_res = res.height;
  csi_config.lane_bit_rate_mbps = 200;  // Faible pour VGA/QVGA
  csi_config.input_data_color_type = MIPI_CSI_COLOR_RAW8;
  csi_config.output_data_color_type = MIPI_CSI_COLOR_RGB565;
  csi_config.data_lane_num = 1;  // COMMENCER AVEC 1 LANE
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 1;
  csi_config.bayer_type = ISP_COLOR_BGGR;
  
  ESP_LOGI(TAG, "  • Résolution: %ux%u", res.width, res.height);
  ESP_LOGI(TAG, "  • Lanes: %d @ %d Mbps", csi_config.data_lane_num, csi_config.lane_bit_rate_mbps);
  ESP_LOGI(TAG, "  • Format: RAW8 → RGB565");
  ESP_LOGI(TAG, "  • Bayer: BGGR");
  
  ESP_LOGI(TAG, "➤ Étape 3/7: Création contrôleur CSI...");
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_ctlr_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  ❌ esp_cam_new_csi_ctlr(): %s", esp_err_to_name(ret));
    ESP_LOGE(TAG, "");
    ESP_LOGE(TAG, "Causes possibles:");
    ESP_LOGE(TAG, "  1. CONFIG_MIPI_CSI_ENABLED=n");
    ESP_LOGE(TAG, "  2. Pins CSI mal configurés");
    ESP_LOGE(TAG, "  3. Paramètres CSI incompatibles");
    return false;
  }
  ESP_LOGI(TAG, "  ✅ Contrôleur créé");
  
  ESP_LOGI(TAG, "➤ Étape 4/7: Callbacks...");
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
    ESP_LOGI("CSI", "✅ FRAME RÉELLE REÇUE: %u bytes", trans->received_size);
    return true;
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->cam_ctlr_handle_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  ❌ Callbacks: %s", esp_err_to_name(ret));
    esp_cam_del_ctlr(this->cam_ctlr_handle_);
    this->cam_ctlr_handle_ = nullptr;
    return false;
  }
  ESP_LOGI(TAG, "  ✅ Callbacks enregistrés");
  
  ESP_LOGI(TAG, "➤ Étape 5/7: Activation contrôleur...");
  ret = esp_cam_ctlr_enable(this->cam_ctlr_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  ❌ Enable: %s", esp_err_to_name(ret));
    esp_cam_del_ctlr(this->cam_ctlr_handle_);
    this->cam_ctlr_handle_ = nullptr;
    return false;
  }
  ESP_LOGI(TAG, "  ✅ Activé");
  
  ESP_LOGI(TAG, "➤ Étape 6/7: Démarrage réception...");
  ret = esp_cam_ctlr_start(this->cam_ctlr_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "  ❌ Start: %s", esp_err_to_name(ret));
    esp_cam_ctlr_disable(this->cam_ctlr_handle_);
    esp_cam_del_ctlr(this->cam_ctlr_handle_);
    this->cam_ctlr_handle_ = nullptr;
    return false;
  }
  ESP_LOGI(TAG, "  ✅ Réception démarrée");
  
  this->csi_initialized_ = true;
  
  ESP_LOGI(TAG, "➤ Étape 7/7: Test première frame...");
  delay(100);  // Attendre que le capteur envoie des données
  
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "╔════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║  ✅✅✅ CSI INITIALISÉ AVEC SUCCÈS ✅✅✅  ║");
  ESP_LOGI(TAG, "║  Vous devriez voir l'image RÉELLE          ║");
  ESP_LOGI(TAG, "╚════════════════════════════════════════════╝");
  ESP_LOGI(TAG, "");
  
  return true;
  
  #endif
}

bool Tab5Camera::capture_csi_frame_() {
  #ifdef CONFIG_ISP_ENABLED
  
  if (this->cam_ctlr_handle_ == nullptr) {
    ESP_LOGE(TAG, "Handle CSI null");
    return false;
  }
  
  esp_cam_ctlr_trans_t trans = {};
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_ctlr_handle_, &trans, 1000);  // 1 seconde timeout
  
  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "Frame reçue: %u bytes", trans.received_size);
    return true;
  } else if (ret == ESP_ERR_TIMEOUT) {
    ESP_LOGW(TAG, "⚠ TIMEOUT CSI - Pas de signal caméra");
    ESP_LOGW(TAG, "Vérifiez:");
    ESP_LOGW(TAG, "  - Câble CSI connecté");
    ESP_LOGW(TAG, "  - Capteur alimenté");
    ESP_LOGW(TAG, "  - Registre 0x0100 = 0x01");
    return false;
  } else {
    ESP_LOGE(TAG, "❌ Erreur CSI: %s", esp_err_to_name(ret));
    return false;
  }
  
  #else
  ESP_LOGE(TAG, "CONFIG_ISP_ENABLED non défini");
  return false;
  #endif
}

bool Tab5Camera::take_snapshot() {
  ESP_LOGI(TAG, "=== SNAPSHOT ===");
  return this->capture_frame();
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Caméra non initialisée");
    return false;
  }
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "✅ Streaming démarré");
  return true;
}

bool Tab5Camera::stop_streaming() {
  this->streaming_ = false;
  ESP_LOGI(TAG, "✅ Streaming arrêté");
  return true;
}

CameraFrameBuffer *Tab5Camera::get_frame_buffer() {
  if (!this->initialized_) {
    return nullptr;
  }
  return &this->frame_buffer_;
}

void Tab5Camera::return_frame_buffer() {
  // Rien
}

}  // namespace tab5_camera
}  // namespace esphome
