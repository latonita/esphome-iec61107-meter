import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from . import IEC61107Component, CONF_IEC61107_ID, iec61107_ns, validate_request_format, CONF_REQUEST

IEC61107Sensor = iec61107_ns.class_("IEC61107Sensor", sensor.Sensor)

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        IEC61107Sensor,
    ).extend(
        {
            cv.GenerateID(CONF_IEC61107_ID): cv.use_id(IEC61107Component),
            cv.Required(CONF_REQUEST): validate_request_format,
        }
    ),
    cv.has_exactly_one_key(CONF_REQUEST),
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_IEC61107_ID])
    var = await sensor.new_sensor(config)


    if CONF_REQUEST in config:
        cg.add(var.set_request(config[CONF_REQUEST]))

    cg.add(component.register_sensor(var))
