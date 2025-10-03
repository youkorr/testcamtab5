import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["esp32"]

esp_cam_sensor_ns = cg.esphome_ns.namespace("esp_cam_sensor_esphome")

CONFIG_SCHEMA = cv.Schema({})

async def to_code(config):
    # Ajouter les includes nécessaires
    cg.add_platformio_option("lib_deps", [])
    
    # Ajouter les flags de compilation
    cg.add_build_flag("-DCONFIG_ESP_CAM_SENSOR_ENABLED=1")
    cg.add_build_flag("-DCONFIG_CAMERA_SC202CS=1")
    cg.add_build_flag("-DCONFIG_ESP_CAM_SENSOR_MIPI_CSI_ENABLED=1")
