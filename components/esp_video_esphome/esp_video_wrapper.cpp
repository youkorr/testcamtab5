#include "esp_video_wrapper.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

// Includes complets ici dans le .cpp
extern "C" {
  #include "esp_video.h"
  //#include "esp_video_init.h"
  //#include "esp_video_device_internal.h"
  #include "../esp_cam_sensor_esphome/esp_cam_sensor.h"
  #include "esp_cam_ctlr_csi.h"
  #include "driver/isp.h"
  #include "esp_ldo_regulator.h"
}

namespace esphome {
namespace esp_video_esphome {

static const char *const TAG = "esp_video";

// Variables statiques
struct esp_video *ESPVideoWrapper::video_device_ = nullptr;
bool ESPVideoWrapper::initialized_ = false;
bool ESPVideoWrapper::streaming_ = false;
esp_ldo_channel_handle_t ESPVideoWrapper::ldo_handle_ = nullptr;

esp_err_t ESPVideoWrapper::init_video_system(esp_cam_sensor_device_t *sensor_device) {
  if (initialized_) {
    ESP_LOGW(TAG, "Système vidéo déjà initialisé");
    return ESP_OK;
  }
  
  ESP_LOGI(TAG, "Initialisation du système vidéo ESP...");
  
  // 1. Initialiser LDO pour MIPI-CSI (requis pour ESP32-P4)
  esp_ldo_channel_config_t ldo_config = {
    .chan_id = 3,  // LDO channel 3 pour MIPI
    .voltage_mv = 2500,
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_config, &ldo_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec init LDO: %d", ret);
    return ret;
  }
  ESP_LOGD(TAG, "✓ LDO initialisé (2.5V)");
  
  // 2. Créer le device vidéo CSI
  ret = esp_video_create_csi_video_device(sensor_device);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec création device CSI: %d", ret);
    esp_ldo_release_channel(ldo_handle_);
    return ret;
  }
  ESP_LOGD(TAG, "✓ Device CSI créé");
  
  // 3. Ouvrir le device vidéo
  video_device_ = esp_video_open("MIPI-CSI");
  if (video_device_ == nullptr) {
    ESP_LOGE(TAG, "Échec ouverture device vidéo");
    esp_ldo_release_channel(ldo_handle_);
    return ESP_FAIL;
  }
  ESP_LOGD(TAG, "✓ Device vidéo ouvert");
  
  initialized_ = true;
  ESP_LOGI(TAG, "✅ Système vidéo initialisé");
  
  return ESP_OK;
}

esp_err_t ESPVideoWrapper::set_format(const VideoConfig &config) {
  if (!initialized_ || video_device_ == nullptr) {
    ESP_LOGE(TAG, "Système vidéo non initialisé");
    return ESP_ERR_INVALID_STATE;
  }
  
  // Configuration format V4L2
  struct v4l2_format format = {};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = config.width;
  format.fmt.pix.height = config.height;
  format.fmt.pix.pixelformat = config.pixel_format;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  
  // Pour RGB565
  if (config.pixel_format == V4L2_PIX_FMT_RGB565) {
    format.fmt.pix.bytesperline = config.width * 2;
    format.fmt.pix.sizeimage = config.width * config.height * 2;
  }
  // Pour RAW8
  else if (config.pixel_format == V4L2_PIX_FMT_SBGGR8) {
    format.fmt.pix.bytesperline = config.width;
    format.fmt.pix.sizeimage = config.width * config.height;
  }
  
  esp_err_t ret = esp_video_set_format(video_device_, &format);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec configuration format: %d", ret);
    return ret;
  }
  
  ESP_LOGI(TAG, "Format configuré: %ux%u, format=0x%08X", 
           config.width, config.height, config.pixel_format);
  
  return ESP_OK;
}

esp_err_t ESPVideoWrapper::start_capture() {
  if (!initialized_ || video_device_ == nullptr) {
    ESP_LOGE(TAG, "Système vidéo non initialisé");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (streaming_) {
    ESP_LOGW(TAG, "Capture déjà démarrée");
    return ESP_OK;
  }
  
  // 1. Setup des buffers
  esp_err_t ret = esp_video_setup_buffer(
    video_device_,
    V4L2_BUF_TYPE_VIDEO_CAPTURE,
    V4L2_MEMORY_MMAP,
    3  // 3 buffers pour le triple buffering
  );
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec setup buffers: %d", ret);
    return ret;
  }
  
  // 2. Queue les buffers
  for (int i = 0; i < 3; i++) {
    ret = esp_video_queue_element_index(video_device_, V4L2_BUF_TYPE_VIDEO_CAPTURE, i);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Échec queue buffer %d: %d", i, ret);
      return ret;
    }
  }
  
  // 3. Démarrer la capture
  ret = esp_video_start_capture(video_device_, V4L2_BUF_TYPE_VIDEO_CAPTURE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec démarrage capture: %d", ret);
    return ret;
  }
  
  streaming_ = true;
  ESP_LOGI(TAG, "✅ Capture démarrée");
  
  return ESP_OK;
}

esp_err_t ESPVideoWrapper::stop_capture() {
  if (!streaming_) {
    return ESP_OK;
  }
  
  esp_err_t ret = esp_video_stop_capture(video_device_, V4L2_BUF_TYPE_VIDEO_CAPTURE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec arrêt capture: %d", ret);
    return ret;
  }
  
  streaming_ = false;
  ESP_LOGI(TAG, "✅ Capture arrêtée");
  
  return ESP_OK;
}

uint8_t* ESPVideoWrapper::get_frame_buffer(size_t *size) {
  if (!streaming_ || video_device_ == nullptr) {
    return nullptr;
  }
  
  // Récupérer un élément du buffer avec timeout de 1 seconde
  struct esp_video_buffer_element *element = 
    esp_video_recv_element(video_device_, V4L2_BUF_TYPE_VIDEO_CAPTURE, pdMS_TO_TICKS(1000));
  
  if (element == nullptr) {
    ESP_LOGV(TAG, "Timeout attente frame");
    return nullptr;
  }
  
  if (size != nullptr) {
    *size = element->valid_size;
  }
  
  return element->buffer;
}

void ESPVideoWrapper::release_frame_buffer(uint8_t *buffer) {
  if (video_device_ == nullptr || buffer == nullptr) {
    return;
  }
  
  // Remettre le buffer dans la queue
  struct esp_video_stream *stream = 
    esp_video_get_stream(video_device_, V4L2_BUF_TYPE_VIDEO_CAPTURE);
  
  if (stream != nullptr && stream->buffer != nullptr) {
    struct esp_video_buffer_element *element = 
      esp_video_buffer_get_element_by_buffer(stream->buffer, buffer);
    
    if (element != nullptr) {
      esp_video_queue_element(video_device_, V4L2_BUF_TYPE_VIDEO_CAPTURE, element);
    }
  }
}

bool ESPVideoWrapper::is_streaming() {
  return streaming_;
}

void ESPVideoWrapper::cleanup() {
  if (streaming_) {
    stop_capture();
  }
  
  if (video_device_ != nullptr) {
    esp_video_close(video_device_);
    video_device_ = nullptr;
  }
  
  if (ldo_handle_ != nullptr) {
    esp_ldo_release_channel(ldo_handle_);
    ldo_handle_ = nullptr;
  }
  
  initialized_ = false;
  ESP_LOGI(TAG, "Système vidéo nettoyé");
}

}  // namespace esp_video_esphome
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
