idf_component_register(
    SRCS 
        "tab5_camera.cpp"
    INCLUDE_DIRS 
        "."
        "../sensors_sc202cs"  # Ajouter cette ligne
    REQUIRES
        esphome
        esp_driver_i2c
        esp_cam_sensor_esphome
        sensors_sc202cs  # Ajouter cette dépendance
        driver
        esp_driver_isp
)
