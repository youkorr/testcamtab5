idf_component_register(
    SRCS "tab5_camera.cpp"
    INCLUDE_DIRS 
        "."
        "${COMPONENT_PATH}/.."
        "${COMPONENT_PATH}/../esp_cam_sensor_esphome/esp_cam_sensor_wrapper.h"
        "${COMPONENT_PATH}/../sensors_sc202cs"

    REQUIRES 
        esp_driver_i2c
        esp_driver_isp
        esp_mm
        esp_driver_cam
        esp_cam_sensor_esphome
        esp_video_esphome
    PRIV_REQUIRES
        esphome
)

