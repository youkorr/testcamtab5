idf_component_register(
    SRCS 
        "esp_cam_sensor.cpp"
        "${COMPONENT_PATH}/../sensors_sc202cs/sc202cs.c"
    INCLUDE_DIRS 
        "."
        "${COMPONENT_PATH}/../esp_cam_sensor"
        "${COMPONENT_PATH}/../sensors_sc202cs"
    REQUIRES
        esp_driver_i2c
        esphome
)

# Définir le symbole pour forcer le linkage du driver SC202CS
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u sc202cs_detect")
