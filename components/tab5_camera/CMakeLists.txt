idf_component_register(
    SRCS "tab5_camera.cpp"
    INCLUDE_DIRS 
        "."                # includes locaux (tab5_camera.h)
        "${COMPONENT_PATH}/.."   # ajoute le dossier parent (components/)
    REQUIRES 
        esp_driver_i2c
        esp_driver_isp
        esp_mm
        esp_driver_cam
        esp_cam_sensor_esphome   # dépendance vers ton autre composant
    PRIV_REQUIRES
        esphome
)

