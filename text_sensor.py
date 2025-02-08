import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, DEVICE_CLASS_TIMESTAMP

from . import CONF_EKHHE_ID, DaikinEkhhe
from .const import CURRENT_TIME


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
        cv.Optional(CURRENT_TIME): text_sensor.text_sensor_schema(
            device_class=DEVICE_CLASS_TIMESTAMP,
        ),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])

    if CURRENT_TIME in config:
        sens = await text_sensor.new_text_sensor(config[CURRENT_TIME])
        cg.add(hub.register_timestamp_sensor(sens))