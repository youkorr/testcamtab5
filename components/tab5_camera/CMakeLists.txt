idf_component_register(
    SRCS 
        "tab5_camera.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES
        esphome
        esp_driver_i2c
        esp_cam_sensor_esphome
        driver
        esp_driver_isp
        sensors_sc202cs
    PRIV_REQUIRES
        sensors_sc202cs
)

# Ajouter le répertoire d'include du composant sensors_sc202cs
target_include_directories(${COMPONENT_LIB} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/../sensors_sc202cs
)
