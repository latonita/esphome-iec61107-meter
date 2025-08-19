import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_INDEX,
)
from . import (
    Iec61107,
    CONF_iec61107_ID,
    iec61107_ns,
    validate_request_format,
    CONF_OBIS_CODE,
    CONF_SUB_INDEX,
    DEFAULTS_MAX_SENSOR_INDEX,
)

AUTO_LOAD = ["iec61107"]

Iec61107TextSensor = iec61107_ns.class_(
    "Iec61107TextSensor", text_sensor.TextSensor
)

CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema(
        Iec61107TextSensor,
    ).extend(
        {
            cv.GenerateID(CONF_iec61107_ID): cv.use_id(Iec61107),
            cv.Required(CONF_OBIS_CODE): validate_request_format,
            cv.Optional(CONF_INDEX, default=1): cv.int_range(
                min=1, max=DEFAULTS_MAX_SENSOR_INDEX
            ),
            cv.Optional(CONF_SUB_INDEX, default=0): cv.int_range(
                min=0, max=255
            ),
        }
    ),
    cv.has_exactly_one_key(CONF_OBIS_CODE),
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_iec61107_ID])
    var = await text_sensor.new_text_sensor(config)

    if CONF_OBIS_CODE in config:
        cg.add(var.set_request(config[CONF_OBIS_CODE]))

    cg.add(var.set_index(config[CONF_INDEX]))
    cg.add(var.set_sub_index(config[CONF_SUB_INDEX]))

    cg.add(component.register_sensor(var))
