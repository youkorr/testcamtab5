idf_component_register(
    SRCS "tab5_camera.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        esp_driver_i2c
        esp_driver_isp
        esp_mm
        esp_driver_cam
        esp_cam_sensor_esphome
    PRIV_REQUIRES
        esphome
)
