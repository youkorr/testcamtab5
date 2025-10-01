import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_FREQUENCY,
)
from esphome import pins

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c", "esp32"]
MULTI_CONF = True

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Configuration pour SC202CS
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_FRAMERATE = "framerate"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_RESET_PIN = "reset_pin"
CONF_ADDRESS_SENSOR_SC202CS = "address_sensor_sc202cs"

# Résolutions SC202CS (capteur 2MP)
CAMERA_RESOLUTIONS = {
    "1080P": 0,
    "720P": 1,
    "VGA": 2,
    "QVGA": 3,
}

# Formats pixel supportés
PIXEL_FORMATS = {
    "RGB565": 0,
    "YUV422": 1,
    "RAW8": 2,
    "JPEG": 3,
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5Camera),
            cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
            cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=47): pins.gpio_output_pin_schema,
            cv.Optional(CONF_FREQUENCY, default=24000000): cv.int_range(min=6000000, max=40000000),
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ADDRESS_SENSOR_SC202CS, default=0x36): cv.i2c_address,
            cv.Optional(CONF_RESOLUTION, default="VGA"): cv.enum(CAMERA_RESOLUTIONS, upper=True),
            cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.enum(PIXEL_FORMATS, upper=True),
            cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
            cv.Optional(CONF_FRAMERATE, default=30): cv.int_range(min=1, max=60),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x36))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    cg.add(var.set_name(config[CONF_NAME]))
    
    ext_clock_pin = await cg.gpio_pin_expression(config[CONF_EXTERNAL_CLOCK_PIN])
    cg.add(var.set_external_clock_pin(ext_clock_pin))
    cg.add(var.set_external_clock_frequency(config[CONF_FREQUENCY]))
    
    cg.add(var.set_sensor_address(config[CONF_ADDRESS_SENSOR_SC202CS]))
    
    cg.add(var.set_resolution(config[CONF_RESOLUTION]))
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))
    
    # Configuration pour ESP32-P4 avec ESP-IDF 5.x
    cg.add_build_flag("-DBOARD_HAS_PSRAM")
    cg.add_build_flag("-DCONFIG_CAMERA_CORE0=1")
    cg.add_build_flag("-DCONFIG_CAMERA_SC202CS=1")
    cg.add_build_flag("-DUSE_ESP32_VARIANT_ESP32P4")
    





