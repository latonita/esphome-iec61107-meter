import re
from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, binary_sensor, time
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_PASSWORD,
    CONF_RECEIVE_TIMEOUT,
    CONF_UPDATE_INTERVAL,
    CONF_FLOW_CONTROL_PIN,
    CONF_TIME_ID,
)

CODEOWNERS = ["@latonita"]

AUTO_LOAD = ["binary_sensor"]

DEPENDENCIES = ["uart"]

MULTI_CONF = True

DEFAULTS_MAX_SENSOR_INDEX = 12
DEFAULTS_BAUD_RATE_HANDSHAKE = 9600
DEFAULTS_BAUD_RATE_SESSION = 9600
DEFAULTS_RECEIVE_TIMEOUT = "500ms"
DEFAULTS_DELAY_BETWEEN_REQUESTS = "50ms"
DEFAULTS_UPDATE_INTERVAL = "30s"

CONF_iec61107_ID = "iec61107_id"
CONF_OBIS_CODE = "obis_code"
CONF_DELAY_BETWEEN_REQUESTS = "delay_between_requests"
CONF_SUB_INDEX = "sub_index"

CONF_INDICATOR = "indicator"
CONF_REBOOT_AFTER_FAILURE = "reboot_after_failure"

CONF_BAUD_RATE_HANDSHAKE = "baud_rate_handshake"

CONF_PROGRAMMING_MODE = "programming_mode"
CONF_CRC_METHOD = "crc_method"


iec61107_ns = cg.esphome_ns.namespace("iec61107")
Iec61107 = iec61107_ns.class_(
    "Iec61107Component", cg.Component, uart.UARTDevice
)

BAUD_RATES = [300, 600, 1200, 2400, 4800, 9600, 19200]

CRC_METHOD = iec61107_ns.enum("IecCrcMethod")
CRC_METHOD_OPTIONS = {
    "SUM7" : CRC_METHOD.CRC_SUM7,
    "XOR"  : CRC_METHOD.CRC_XOR,
}


def validate_request_format(value):
    if not value.endswith(")"):
        value += "()"

    pattern = r"\b[a-zA-Z0-9_][a-zA-Z0-9_]*\s*\([^)]*\)$"
    if not re.match(pattern, value):
        raise cv.Invalid(
            "Invalid request format. Proper is 'REQUEST' or 'REQUEST()' or 'REQUEST(ARGS)'"
        )

    if len(value) > 15:
        raise cv.Invalid(
            "Request length must be no longer than 15 characters including ()"
        )
    return value


def validate_meter_address(value):
    if len(value) > 15:
        raise cv.Invalid("Meter address length must be no longer than 15 characters")
    return value

def validate_meter_password(value):
    if len(value) > 8:
        raise cv.Invalid("Meter password must be no longer than 8 characters")
    return value

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Iec61107),
            cv.Optional(CONF_ADDRESS, default=""): cv.All(
                cv.string, validate_meter_address
            ),
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(
                CONF_BAUD_RATE_HANDSHAKE, default=DEFAULTS_BAUD_RATE_HANDSHAKE
            ): cv.one_of(*BAUD_RATES),
            cv.Optional(
                CONF_RECEIVE_TIMEOUT, default=DEFAULTS_RECEIVE_TIMEOUT
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_DELAY_BETWEEN_REQUESTS, default=DEFAULTS_DELAY_BETWEEN_REQUESTS
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_UPDATE_INTERVAL, default=DEFAULTS_UPDATE_INTERVAL
            ): cv.update_interval,
            cv.Optional(CONF_REBOOT_AFTER_FAILURE, default=0): cv.int_range(
                min=0, max=100
            ),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_PROGRAMMING_MODE, default=False) : cv.boolean,
            cv.Optional(CONF_PASSWORD, default="00000000"): cv.All(
                cv.string, validate_meter_password
            ),
            cv.Optional(CONF_CRC_METHOD, default="SUM7"): cv.enum(CRC_METHOD_OPTIONS),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if flow_control_pin := config.get(CONF_FLOW_CONTROL_PIN):
        pin = await cg.gpio_pin_expression(flow_control_pin)
        cg.add(var.set_flow_control_pin(pin))

    if indicator_config := config.get(CONF_INDICATOR):
        sens = cg.new_Pvariable(indicator_config[CONF_ID])
        await binary_sensor.register_binary_sensor(sens, indicator_config)
        cg.add(var.set_indicator(sens))

    if CONF_TIME_ID in config:
        time_ = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_source(time_))
        
    cg.add(var.set_meter_address(config[CONF_ADDRESS]))
    cg.add(var.set_baud_rate_for_handshake(config[CONF_BAUD_RATE_HANDSHAKE]))
    cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT]))
    cg.add(var.set_delay_between_requests_ms(config[CONF_DELAY_BETWEEN_REQUESTS]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_reboot_after_failure(config[CONF_REBOOT_AFTER_FAILURE]))
    cg.add(var.set_programming_mode_required(config[CONF_PROGRAMMING_MODE]))
    cg.add(var.set_password(config[CONF_PASSWORD]))
    cg.add(var.set_crc_method(config[CONF_CRC_METHOD]))
