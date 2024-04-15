import re
from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, binary_sensor, time
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_BAUD_RATE,
    CONF_RECEIVE_TIMEOUT,
    CONF_UPDATE_INTERVAL,
    CONF_FLOW_CONTROL_PIN,
    CONF_TIME_ID,
    DEVICE_CLASS_PROBLEM,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

CODEOWNERS = ["@latonita"]

AUTO_LOAD = ["binary_sensor"]

DEPENDENCIES = ["uart"]

MAX_SENSOR_INDEX = 12

CONF_IEC61107_ID = "iec61107_id"
CONF_REQUEST = "request"
CONF_DELAY_BETWEEN_REQUESTS = "delay_between_requests"

CONF_INDICATOR = "indicator"
CONF_REBOOT_AFTER_FAILURE = "reboot_after_failure"

CONF_BAUD_RATE_HANDSHAKE = "baud_rate_handshake"
# CONF_STAT_ERR_CRC = "stat_err_crc"

iec61107_ns = cg.esphome_ns.namespace("iec61107")
IEC61107Component = iec61107_ns.class_(
    "IEC61107Component", cg.Component, uart.UARTDevice
)

BAUD_RATES = [300, 600, 1200, 2400, 4800, 9600, 19200]


def validate_request_format(value):
    if not value.endswith(")"):
        value += "()"

    pattern = r"\b[a-zA-Z_][a-zA-Z0-9_]*\s*\([^)]*\)$"
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


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IEC61107Component),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_ADDRESS, default=""): cv.All(
                cv.string, validate_meter_address
            ),
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_BAUD_RATE_HANDSHAKE, default=9600): cv.one_of(*BAUD_RATES),
            cv.Optional(CONF_BAUD_RATE, default=9600): cv.one_of(*BAUD_RATES),
            cv.Optional(
                CONF_RECEIVE_TIMEOUT, default="500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_DELAY_BETWEEN_REQUESTS, default="150ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.update_interval,
            cv.Optional(CONF_REBOOT_AFTER_FAILURE, default=0): cv.int_range(
                min=0, max=100
            ),
            cv.Optional(CONF_INDICATOR): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            # cv.Optional(CONF_STAT_ERR_CRC): sensor.sensor_schema(
            #     entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            #     accuracy_decimals=2,
            # ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))

    # if stat_config := config.get(CONF_STAT_ERR_CRC):
    #     sens = await sensor.new_sensor(stat_config)
    #     cg.add(var.set_stat_err_crc(sens))

    if indicator_config := config.get(CONF_INDICATOR):
        sens = cg.new_Pvariable(indicator_config[CONF_ID])
        await binary_sensor.register_binary_sensor(sens, indicator_config)
        cg.add(var.set_indicator(sens))

    # if err_stat_config := config.get(CONF_STAT_ERR_CRC):
    #     sens = cg.new_Pvariable(err_stat_config[CONF_ID])
    #     await sensor.register_sensor(sens, err_stat_config)
    #     cg.add(var.set_stat_err_crc(sens))

    # if CONF_INDICATOR in config:
    #     conf = config[CONF_INDICATOR]
    #     sens = cg.new_Pvariable(conf[CONF_ID])
    #     await binary_sensor.register_binary_sensor(sens, conf)
    #     cg.add(var.set_indicator(sens))

    cg.add(var.set_meter_address(config[CONF_ADDRESS]))
    cg.add(var.set_baud_rates(config[CONF_BAUD_RATE_HANDSHAKE], config[CONF_BAUD_RATE]))
    cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT]))
    cg.add(var.set_delay_between_requests_ms(config[CONF_DELAY_BETWEEN_REQUESTS]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_reboot_after_failure(config[CONF_REBOOT_AFTER_FAILURE]))

    if CONF_TIME_ID in config:
        time_ = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time(time_))
