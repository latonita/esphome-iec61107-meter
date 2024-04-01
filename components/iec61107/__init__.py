import re
from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_RECEIVE_TIMEOUT,
    CONF_UPDATE_INTERVAL,
    CONF_FLOW_CONTROL_PIN,
    CONF_INDEX,
)

CODEOWNERS = ["@latonita"]

AUTO_LOAD = ["sensor"]

DEPENDENCIES = ["uart"]

CONF_IEC61107_ID = "iec61107_id"
CONF_REQUEST = "request"
CONF_READOUT_ENABLED = "readout_enabled"

iec61107_ns = cg.esphome_ns.namespace("iec61107")
IEC61107Component = iec61107_ns.class_(
    "IEC61107Component", cg.Component, uart.UARTDevice
)


def validate_request_format(value):
# tbd
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IEC61107Component),
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_RECEIVE_TIMEOUT, default="500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.update_interval,
            cv.Optional(CONF_READOUT_ENABLED, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_UPDATE_INTERVAL in config:
        cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    if CONF_RECEIVE_TIMEOUT in config:
        cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT]))

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))

    if CONF_READOUT_ENABLED in config:
        cg.add(var.set_enable_readout(config[CONF_READOUT_ENABLED]))
