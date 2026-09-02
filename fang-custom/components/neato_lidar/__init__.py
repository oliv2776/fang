import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, mqtt
from esphome.const import CONF_ID, CONF_DEVICE_CLASS

AUTO_LOAD = ["uart", "mqtt"]

DEPENDENCIES = ["uart", "mqtt"]

neato_lidar_ns = cg.esphome_ns.namespace("neato_lidar")
NeatoLidarComponent = neato_lidar_ns.class_(
    "NeatoLidarComponent",
    cg.PollingComponent,
    uart.UARTDevice,
    mqtt.MQTTComponent,
    auto_unload=False,
)

CONF_DEVICE_ID = "device_id"
CONF_MQTT_TOPIC = "mqtt_topic"
CONF_SCAN_INTERVAL = "scan_interval"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NeatoLidarComponent),
        cv.Required(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_MQTT_TOPIC, default="neato/scan"): cv.string,
        cv.Optional(CONF_SCAN_INTERVAL, default="100ms"): cv.positive_time_period_milliseconds,
    }
).extend(uart.UART_DEVICE_SCHEMA)


def to_code(config, cg):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var)

    # UART
    uart_parent = config.get("uart_id")
    if uart_parent:
        uart_comp = cg.variables[uart_parent]
        cg.add(var, uart_comp)

    # MQTT
    mqtt_parent = config.get("mqtt_id")
    if mqtt_parent:
        mqtt_comp = cg.variables[mqtt_parent]
        cg.add(var, mqtt_comp)

    # Paramètres
    cg.add(var, "set_device_id", config[CONF_DEVICE_ID])
    cg.add(var, "set_mqtt_topic", config[CONF_MQTT_TOPIC])
    cg.add(var, "set_scan_interval", config[CONF_SCAN_INTERVAL])

    # Polling interval
    cg.add(var, "set_update_interval", config[CONF_SCAN_INTERVAL])

    return var
