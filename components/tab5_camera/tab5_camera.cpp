#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <driver/ledc.h>

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

// Tables de configuration SC202CS pour différentes résolutions
// Ces valeurs proviennent du driver officiel M5Tab5

// Configuration 1080P (1920x1080)
static const uint16_t SC202CS_1080P_REGS[][2] = {
    {0x0103, 0x01}, // Soft reset
    {0x0100, 0x00}, // Sleep mode
    {0x36e9, 0x80}, // PLL
    {0x37f9, 0x80}, // PLL
    // Configuration résolution 1080P
    {0x3200, 0x00}, // X start high
    {0x3201, 0x00}, // X start low
    {0x3202, 0x00}, // Y start high
    {0x3203, 0x00}, // Y start low
    {0x3204, 0x07}, // X end high
    {0x3205, 0x8f}, // X end low
    {0x3206, 0x04}, // Y end high
    {0x3207, 0x47}, // Y end low
    {0x3208, 0x07}, // X output size high
    {0x3209, 0x80}, // X output size low (1920)
    {0x320a, 0x04}, // Y output size high
    {0x320b, 0x38}, // Y output size low (1080)
    // Timing
    {0x320c, 0x04}, // HTS high
    {0x320d, 0x4c}, // HTS low
    {0x320e, 0x04}, // VTS high
    {0x320f, 0x66}, // VTS low
    // Autres paramètres
    {0x3301, 0x06},
    {0x3304, 0x50},
    {0x3306, 0x48},
    {0x3308, 0x18},
    {0x3309, 0x68},
    {0x330b, 0xe8},
    {0x330d, 0x28},
    {0x330e, 0x48},
    {0x3314, 0x94},
    {0x331e, 0x41},
    {0x331f, 0x61},
    {0x3333, 0x10},
    {0x3334, 0x40},
    {0x335e, 0x06},
    {0x335f, 0x0a},
    {0x3364, 0x5e},
    {0x337c, 0x02},
    {0x337d, 0x0a},
    {0x3390, 0x01},
    {0x3391, 0x03},
    {0x3392, 0x07},
    {0x3393, 0x06},
    {0x3394, 0x06},
    {0x3395, 0x06},
    {0x3396, 0x48},
    {0x3397, 0x49},
    {0x3398, 0x5b},
    {0x3399, 0x06},
    {0x339a, 0x0a},
    {0x339b, 0x10},
    {0x339c, 0x22},
    {0x33a2, 0x04},
    {0x33ad, 0x0c},
    {0x33b1, 0x80},
    {0x33b3, 0x30},
    {0x33f9, 0x68},
    {0x33fb, 0x90},
    {0x33fc, 0x48},
    {0x33fd, 0x5f},
    {0x349f, 0x03},
    {0x34a6, 0x48},
    {0x34a7, 0x5f},
    {0x34a8, 0x30},
    {0x34a9, 0x30},
    {0x34aa, 0x00},
    {0x34ab, 0xe8},
    {0x34ac, 0x01},
    {0x34ad, 0x00},
    {0x3630, 0xf0},
    {0x3633, 0x33},
    {0x3634, 0x64},
    {0x3637, 0x50},
    {0x3638, 0x24},
    {0x363a, 0x84},
    {0x363b, 0x04},
    {0x363c, 0x08},
    {0x3641, 0x38},
    {0x3670, 0x4e},
    {0x3674, 0xc0},
    {0x3675, 0xa0},
    {0x3676, 0xa0},
    {0x3677, 0x84},
    {0x3678, 0x88},
    {0x3679, 0x8c},
    {0x367c, 0x48},
    {0x367d, 0x49},
    {0x367e, 0x48},
    {0x367f, 0x5b},
    {0x3690, 0x33},
    {0x3691, 0x33},
    {0x3692, 0x44},
    {0x369c, 0x48},
    {0x369d, 0x5f},
    {0x36ea, 0x35},
    {0x36eb, 0x04},
    {0x36ec, 0x03},
    {0x36ed, 0x24},
    {0x36fa, 0x35},
    {0x36fb, 0x00},
    {0x36fc, 0x10},
    {0x36fd, 0x24},
    {0x3904, 0x04},
    {0x3908, 0x41},
    {0x391d, 0x04},
    {0x39c2, 0x30},
    {0x3e01, 0x8c},
    {0x3e02, 0x20},
    {0x3e16, 0x00},
    {0x3e17, 0x80},
    {0x4500, 0x88},
    {0x4509, 0x20},
    {0x4837, 0x1e},
    {0x5799, 0x00},
    {0x59e0, 0x60},
    {0x59e1, 0x08},
    {0x59e2, 0x3f},
    {0x59e3, 0x18},
    {0x59e4, 0x18},
    {0x59e5, 0x3f},
    {0x59e6, 0x06},
    {0x59e7, 0x02},
    {0x59e8, 0x38},
    {0x59e9, 0x10},
    {0x59ea, 0x0c},
    {0x59eb, 0x10},
    {0x59ec, 0x04},
    {0x59ed, 0x02},
    {0x0100, 0x01}, // Wake up
};

// Configuration 720P (1280x720)
static const uint16_t SC202CS_720P_REGS[][2] = {
    {0x0103, 0x01},
    {0x0100, 0x00},
    {0x36e9, 0x80},
    {0x37f9, 0x80},
    {0x3200, 0x00},
    {0x3201, 0xa0},
    {0x3202, 0x00},
    {0x3203, 0xf0},
    {0x3204, 0x06},
    {0x3205, 0xef},
    {0x3206, 0x03},
    {0x3207, 0x57},
    {0x3208, 0x05}, // 1280
    {0x3209, 0x00},
    {0x320a, 0x02}, // 720
    {0x320b, 0xd0},
    {0x320c, 0x04},
    {0x320d, 0x4c},
    {0x320e, 0x03},
    {0x320f, 0x00},
    // Configuration similaire à 1080P mais adaptée
    {0x3301, 0x06},
    {0x3304, 0x50},
    {0x3306, 0x48},
    {0x3630, 0xf0},
    {0x3633, 0x33},
    {0x3634, 0x64},
    {0x4837, 0x1e},
    {0x0100, 0x01},
};

// Configuration VGA (640x480)
static const uint16_t SC202CS_VGA_REGS[][2] = {
    {0x0103, 0x01},
    {0x0100, 0x00},
    {0x36e9, 0x80},
    {0x37f9, 0x80},
    {0x3200, 0x01},
    {0x3201, 0x48},
    {0x3202, 0x01},
    {0x3203, 0x38},
    {0x3204, 0x06},
    {0x3205, 0x47},
    {0x3206, 0x03},
    {0x3207, 0x0f},
    {0x3208, 0x02}, // 640
    {0x3209, 0x80},
    {0x320a, 0x01}, // 480
    {0x320b, 0xe0},
    {0x320c, 0x04},
    {0x320d, 0x4c},
    {0x320e, 0x02},
    {0x320f, 0x08},
    {0x3301, 0x06},
    {0x3304, 0x50},
    {0x3630, 0xf0},
    {0x4837, 0x1e},
    {0x0100, 0x01},
};

// Configuration QVGA (320x240)
static const uint16_t SC202CS_QVGA_REGS[][2] = {
    {0x0103, 0x01},
    {0x0100, 0x00},
    {0x36e9, 0x80},
    {0x37f9, 0x80},
    {0x3200, 0x01},
    {0x3201, 0x48},
    {0x3202, 0x01},
    {0x3203, 0x38},
    {0x3204, 0x06},
    {0x3205, 0x47},
    {0x3206, 0x03},
    {0x3207, 0x0f},
    {0x3208, 0x01}, // 320
    {0x3209, 0x40},
    {0x320a, 0x00}, // 240
    {0x320b, 0xf0},
    {0x320c, 0x04},
    {0x320d, 0x4c},
    {0x320e, 0x01},
    {0x320f, 0x04},
    {0x3301, 0x06},
    {0x3304, 0x50},
    {0x3630, 0xf0},
    {0x4837, 0x1e},
    {0x0100, 0x01},
};

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Configuration Tab5 Camera...");
  
  // Initialisation du clock externe
  if (!this->start_external_clock_()) {
    ESP_LOGE(TAG, "Échec de l'initialisation du clock externe");
    this->mark_failed();
    return;
  }
  
  // Reset du capteur si pin disponible
  if (this->reset_pin_ != nullptr) {
    if (!this->reset_sensor_()) {
      ESP_LOGE(TAG, "Échec du reset du capteur");
      this->mark_failed();
      return;
    }
  }
  
  // Attente de stabilisation
  delay(50);
  
  // Initialisation du capteur SC202CS
  if (!this->init_sc202cs_sensor_()) {
    ESP_LOGE(TAG, "Échec de l'initialisation du capteur SC202CS");
    this->mark_failed();
    return;
  }
  
  // Configuration du capteur
  if (!this->configure_sc202cs_()) {
    ESP_LOGE(TAG, "Échec de la configuration du capteur");
    this->mark_failed();
    return;
  }
  
  // Allocation du buffer de frame
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
  
  // Configuration LEDC pour générer le clock externe
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_2_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = this->xclk_frequency_,
      .clk_cfg = LEDC_AUTO_CLK
  };
  
  esp_err_t err = ledc_timer_config(&ledc_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Échec configuration timer LEDC: %d", err);
    return false;
  }
  
  ledc_channel_config_t ledc_channel = {
      .gpio_num = (int)this->xclk_pin_->get_pin(),
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 2,
      .hpoint = 0
  };
  
  err = ledc_channel_config(&ledc_channel);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Échec configuration canal LEDC: %d", err);
    return false;
  }
  
  ESP_LOGI(TAG, "Clock externe démarré à %u Hz sur GPIO%d", 
           this->xclk_frequency_, this->xclk_pin_->get_pin());
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
  // Vérifier l'ID du chip SC202CS
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
  
  if (chip_id != SC202CS_CHIP_ID_VALUE) {
    ESP_LOGW(TAG, "Chip ID inattendu (attendu: 0x%04X, reçu: 0x%04X)", 
             SC202CS_CHIP_ID_VALUE, chip_id);
    // On continue quand même, le chip ID peut varier selon les versions
  }
  
  return true;
}

bool Tab5Camera::configure_sc202cs_() {
  // Soft reset du capteur
  if (!this->write_sensor_reg_(SC202CS_RESET_REG, 0x01)) {
    ESP_LOGE(TAG, "Échec du soft reset");
    return false;
  }
  delay(100);
  
  // Charger la table de configuration selon la résolution
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
    delay(1);  // Petit délai entre les écritures
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
  
  // Calculer la taille du buffer selon le format
  switch (this->pixel_format_) {
    case PIXEL_FORMAT_RGB565:
    case PIXEL_FORMAT_YUV422:
      buffer_size = res_info.width * res_info.height * 2;
      break;
    case PIXEL_FORMAT_RAW8:
      buffer_size = res_info.width * res_info.height;
      break;
    case PIXEL_FORMAT_JPEG:
      buffer_size = res_info.width * res_info.height / 2;  // Estimation
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
  
  // TODO: Implémenter la capture via DVP/CSI selon le hardware Tab5
  // Pour l'instant, retourner false
  ESP_LOGW(TAG, "capture_frame() pas encore implémenté");
  return false;
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




