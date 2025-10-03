import esphome.codegen as cg
import esphome.config_validation as cv

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["esp32"]

esp_video_ns = cg.esphome_ns.namespace("esp_video_esphome")

CONFIG_SCHEMA = cv.Schema({})

async def to_code(config):
    # Flags pour activer le support vidéo ESP-IDF
    cg.add_build_flag("-DCONFIG_ESP_VIDEO_ENABLED=1")
    cg.add_build_flag("-DCONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE=1")
    cg.add_build_flag("-DCONFIG_ESP_VIDEO_ENABLE_ISP=1")
    cg.add_build_flag("-DCONFIG_ESP_VIDEO_DISABLE_MIPI_CSI_DRIVER_BACKUP_BUFFER=1")
