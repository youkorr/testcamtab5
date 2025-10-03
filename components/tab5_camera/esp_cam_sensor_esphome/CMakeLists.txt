# components/esp_cam_sensor_esphome/CMakeLists.txt
idf_component_register(
    SRCS 
        "esp_cam_sensor_wrapper.cpp"
        "${COMPONENT_PATH}/../sensors_sc202cs/sc202cs.c"
    INCLUDE_DIRS 
        "."
        "${COMPONENT_PATH}/../esp_cam_sensor"
        "${COMPONENT_PATH}/../sensors_sc202cs"
    REQUIRES
        esp_driver_i2c
        esphome
)

target_link_libraries(${COMPONENT_LIB} INTERFACE "-u sc202cs_detect")
