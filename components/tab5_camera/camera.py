"""Support pour l'interface caméra ESPHome avec Tab5."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import camera
from esphome.const import CONF_ID

from . import Tab5Camera, tab5_camera_ns, CONF_RESOLUTION, CONF_PIXEL_FORMAT

DEPENDENCIES = ["tab5_camera"]

Tab5CameraSensor = tab5_camera_ns.class_(
    "Tab5CameraSensor", camera.Camera, cg.PollingComponent
)

CONFIG_SCHEMA = cv.All(
    camera.CAMERA_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(Tab5CameraSensor),
            cv.GenerateID(Tab5Camera): cv.use_id(Tab5Camera),
        }
    ).extend(cv.polling_component_schema("1s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await camera.register_camera(var, config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[Tab5Camera])
    cg.add(var.set_parent(parent))
