import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_INDEX,
)
from . import (
    IEC61107Component,
    CONF_IEC61107_ID,
    iec61107_ns,
    CONF_REQUEST,
    validate_request_format,
)

IEC61107Sensor = iec61107_ns.class_("IEC61107Sensor", sensor.Sensor)


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        IEC61107Sensor,
    ).extend(
        {
            cv.GenerateID(CONF_IEC61107_ID): cv.use_id(IEC61107Component),
            cv.Required(CONF_REQUEST): validate_request_format,
            cv.Optional(CONF_INDEX, default=1): cv.int_range(min=1, max=12),
        }
    ),
    cv.has_exactly_one_key(CONF_REQUEST),
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_IEC61107_ID])
    var = await sensor.new_sensor(config)

    if CONF_REQUEST in config:
        cg.add(var.set_request(config[CONF_REQUEST]))

    cg.add(var.set_index(config[CONF_INDEX]))
    cg.add(component.register_sensor(var))
