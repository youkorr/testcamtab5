idf_component_register(
    SRCS 
        "tab5_camera.cpp"
        "tab5_camera_sensor_impl.cpp"  # Ajouter comme source séparée
    INCLUDE_DIRS 
        "."
    REQUIRES
        esphome
        esp_driver_i2c
        esp_cam_sensor_esphome
        driver
        esp_driver_isp
)
