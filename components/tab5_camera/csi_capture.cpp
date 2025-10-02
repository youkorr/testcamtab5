// Implémentation de la capture CSI pour remplacer le pattern de test
// À ajouter dans tab5_camera.cpp

// Includes nécessaires pour CSI
#include "driver/isp.h"
#include "esp_cam_sensor.h"
#include "driver/isp_types.h"

// Dans la classe Tab5Camera, ajouter ces membres privés:
// esp_cam_sensor_device_t *cam_sensor_{nullptr};
// isp_proc_handle_t isp_proc_{nullptr};
// bool csi_initialized_{false};

// Remplacer la fonction capture_frame() par celle-ci :
bool Tab5Camera::capture_frame() {
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Caméra non initialisée");
    return false;
  }
  
  // Si CSI n'est pas initialisé, essayer de l'initialiser
  if (!this->csi_initialized_) {
    if (!this->init_csi_interface_()) {
      // Si échec CSI, utiliser le pattern de test
      ESP_LOGW(TAG, "CSI non disponible, utilisation du pattern de test");
      return this->generate_test_pattern_();
    }
  }
  
  // Capturer via CSI
  return this->capture_csi_frame_();
}

// Nouvelle fonction pour initialiser l'interface CSI
bool Tab5Camera::init_csi_interface_() {
  ESP_LOGI(TAG, "Initialisation interface CSI...");
  
  // Configuration du capteur via esp_cam_sensor
  esp_cam_sensor_config_t cam_config = {
    .sccb_handle = nullptr,  // Nous gérons I2C nous-mêmes
    .reset_pin = -1,         // Reset géré par nous
    .pwdn_pin = -1,          // Pas de power down
    .xclk_pin = 36,          // Notre GPIO MCLK
    .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
  };
  
  // Les pins CSI sont câblés en dur sur ESP32-P4
  esp_cam_sensor_mipi_csi_config_t csi_config = {
    .lane_num = 2,           // SC2356 utilise 2 lanes
    .clk_freq = 24000000,    // 24 MHz
    .lane_bit_rate_mbps = 800, // 800 Mbps par lane
  };
  
  // Créer le device sensor
  esp_err_t ret = esp_cam_sensor_init(&cam_config, &this->cam_sensor_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec init cam_sensor: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Configurer CSI
  ret = esp_cam_sensor_set_mipi_csi_config(this->cam_sensor_, &csi_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec config CSI: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Configurer le format de sortie
  esp_cam_sensor_format_t format = {
    .format = ESP_CAM_SENSOR_PIXFORMAT_RGB565,
    .width = this->frame_buffer_.width,
    .height = this->frame_buffer_.height,
  };
  
  ret = esp_cam_sensor_set_format(this->cam_sensor_, &format);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec config format: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Démarrer le capteur
  ret = esp_cam_sensor_start(this->cam_sensor_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec démarrage capteur: %s", esp_err_to_name(ret));
    return false;
  }
  
  this->csi_initialized_ = true;
  ESP_LOGI(TAG, "Interface CSI initialisée avec succès");
  return true;
}

// Nouvelle fonction pour capturer une frame via CSI
bool Tab5Camera::capture_csi_frame_() {
  if (this->cam_sensor_ == nullptr) {
    return false;
  }
  
  // Obtenir une frame du capteur
  esp_cam_sensor_frame_t *frame = nullptr;
  esp_err_t ret = esp_cam_sensor_get_frame(this->cam_sensor_, &frame, portMAX_DELAY);
  
  if (ret != ESP_OK || frame == nullptr) {
    ESP_LOGE(TAG, "Échec capture frame: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Copier les données dans notre buffer
  size_t copy_size = std::min(frame->size, this->frame_buffer_.length);
  memcpy(this->frame_buffer_.buffer, frame->data, copy_size);
  
  // Libérer la frame
  esp_cam_sensor_return_frame(this->cam_sensor_, frame);
  
  ESP_LOGV(TAG, "Frame CSI capturée: %u bytes", copy_size);
  return true;
}

// Garder la fonction pattern de test comme fallback
bool Tab5Camera::generate_test_pattern_() {
  // Code du pattern de test actuel (déjà présent)
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
  
  return true;
}
