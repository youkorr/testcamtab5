#include "tab5_camera.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

#define VIDEO_DEVICE_PATH "/dev/video0"
#define BUFFER_COUNT 2

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Initialisation Tab5 Camera avec esp_video...");
  
  if (!this->init_esp_video_()) {
    ESP_LOGE(TAG, "Échec init esp_video");
    this->mark_failed();
    return;
  }
  
  if (!this->open_v4l2_device_()) {
    ESP_LOGE(TAG, "Échec ouverture V4L2");
    this->mark_failed();
    return;
  }
  
  if (!this->init_v4l2_buffers_()) {
    ESP_LOGE(TAG, "Échec init buffers V4L2");
    this->mark_failed();
    return;
  }
  
  if (!this->start_v4l2_stream_()) {
    ESP_LOGE(TAG, "Échec démarrage stream V4L2");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "Caméra Tab5 initialisée avec esp_video");
}

bool Tab5Camera::init_esp_video_() {
  // Configuration CSI selon M5Stack
  esp_video_init_csi_config_t csi_config = {
    .sccb_config = {
      .init_sccb = true,
      .i2c_handle = nullptr,  // nullptr pour auto-init
      .freq = 400000,
    },
    .reset_pin = -1,  // Géré par GPIO expander
    .pwdn_pin = -1,
  };
  
  esp_video_init_config_t video_config = {
    .csi = &csi_config,
    .dvp = nullptr,
    .jpeg = nullptr,
  };
  
  esp_err_t ret = esp_video_init(&video_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init échoué: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "esp_video initialisé");
  return true;
}

bool Tab5Camera::open_v4l2_device_() {
  this->video_fd_ = open(VIDEO_DEVICE_PATH, O_RDONLY);
  if (this->video_fd_ < 0) {
    ESP_LOGE(TAG, "Impossible d'ouvrir %s", VIDEO_DEVICE_PATH);
    return false;
  }
  
  // Obtenir les capacités
  struct v4l2_capability cap;
  if (ioctl(this->video_fd_, VIDIOC_QUERYCAP, &cap) != 0) {
    ESP_LOGE(TAG, "VIDIOC_QUERYCAP échoué");
    close(this->video_fd_);
    return false;
  }
  
  ESP_LOGI(TAG, "V4L2 driver: %s", cap.driver);
  ESP_LOGI(TAG, "V4L2 card: %s", cap.card);
  
  // Configurer le format
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  if (ioctl(this->video_fd_, VIDIOC_G_FMT, &fmt) != 0) {
    ESP_LOGE(TAG, "VIDIOC_G_FMT échoué");
    close(this->video_fd_);
    return false;
  }
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  // Définir RGB565
  fmt.fmt.pix.width = res.width;
  fmt.fmt.pix.height = res.height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
  
  if (ioctl(this->video_fd_, VIDIOC_S_FMT, &fmt) != 0) {
    ESP_LOGE(TAG, "VIDIOC_S_FMT échoué");
    close(this->video_fd_);
    return false;
  }
  
  ESP_LOGI(TAG, "Format V4L2: %ux%u RGB565", res.width, res.height);
  return true;
}

bool Tab5Camera::init_v4l2_buffers_() {
  // Demander des buffers
  struct v4l2_requestbuffers req = {};
  req.count = BUFFER_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  
  if (ioctl(this->video_fd_, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "VIDIOC_REQBUFS échoué");
    return false;
  }
  
  // Mapper les buffers
  for (int i = 0; i < BUFFER_COUNT; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    
    if (ioctl(this->video_fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QUERYBUF échoué");
      return false;
    }
    
    this->mmap_buffers_[i] = (uint8_t*)mmap(
      nullptr, 
      buf.length, 
      PROT_READ | PROT_WRITE, 
      MAP_SHARED, 
      this->video_fd_, 
      buf.m.offset
    );
    
    if (this->mmap_buffers_[i] == MAP_FAILED) {
      ESP_LOGE(TAG, "mmap échoué pour buffer %d", i);
      return false;
    }
    
    // Queue le buffer
    if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QBUF échoué");
      return false;
    }
  }
  
  // Créer notre buffer frame
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_.width = res.width;
  this->frame_buffer_.height = res.height;
  this->frame_buffer_.length = res.width * res.height * 2;
  this->frame_buffer_.buffer = (uint8_t*)heap_caps_malloc(
    this->frame_buffer_.length, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (!this->frame_buffer_.buffer) {
    ESP_LOGE(TAG, "Allocation buffer échouée");
    return false;
  }
  
  ESP_LOGI(TAG, "Buffers V4L2 initialisés");
  return true;
}

bool Tab5Camera::start_v4l2_stream_() {
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  if (ioctl(this->video_fd_, VIDIOC_STREAMON, &type) != 0) {
    ESP_LOGE(TAG, "VIDIOC_STREAMON échoué");
    return false;
  }
  
  ESP_LOGI(TAG, "Stream V4L2 démarré");
  return true;
}

bool Tab5Camera::capture_v4l2_frame_() {
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  
  // Dequeue
  if (ioctl(this->video_fd_, VIDIOC_DQBUF, &buf) != 0) {
    ESP_LOGV(TAG, "VIDIOC_DQBUF échoué");
    return false;
  }
  
  // Copier dans notre buffer
  memcpy(this->frame_buffer_.buffer, this->mmap_buffers_[buf.index], this->frame_buffer_.length);
  
  // Requeue
  if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
    ESP_LOGE(TAG, "VIDIOC_QBUF échoué");
    return false;
  }
  
  return true;
}

bool Tab5Camera::capture_frame() {
  if (!this->initialized_) {
    return false;
  }
  
  return this->capture_v4l2_frame_();
}

bool Tab5Camera::take_snapshot() {
  return this->capture_frame();
}

bool Tab5Camera::start_streaming() {
  this->streaming_ = true;
  return true;
}

bool Tab5Camera::stop_streaming() {
  this->streaming_ = false;
  return true;
}

CameraResolutionInfo Tab5Camera::get_resolution_info_() {
  switch (this->resolution_) {
    case RESOLUTION_720P: return {1280, 720};
    case RESOLUTION_VGA: return {640, 480};
    case RESOLUTION_QVGA: return {320, 240};
    default: return {640, 480};
  }
}

void Tab5Camera::loop() {}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera (esp_video):");
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u", this->frame_buffer_.width, this->frame_buffer_.height);
  ESP_LOGCONFIG(TAG, "  Device: %s", VIDEO_DEVICE_PATH);
}

}  // namespace tab5_camera
}  // namespace esphome





