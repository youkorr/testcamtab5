#pragma once

#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "tab5_camera.h"

namespace esphome {
namespace tab5_camera {

class Tab5CameraSensor : public camera::Camera, public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  
  void set_parent(Tab5Camera *parent) { this->parent_ = parent; }
  
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void capture_and_publish_();
  
  Tab5Camera *parent_{nullptr};
};

}  // namespace tab5_camera
}  // namespace esphome
