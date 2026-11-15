import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart, select, binary_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ['uart']
AUTO_LOAD = ['select', 'binary_sensor']
ns = cg.esphome_ns.namespace('gree_yunyi2')
GreeYunyi2 = ns.class_('GreeYunyi2', cg.Component, climate.Climate, uart.UARTDevice)
GreeSelect = ns.class_('GreeSelect', select.Select)
OPTIONS = {
    'aux_heat': ['开', '关', '空白', '非制热模式', '未知组合'],
    'vertical_position': ['关闭', '扫风', '位置2', '位置3', '位置4', '位置5', '位置6'],
    'horizontal_position': ['关闭', '扫风', '位置2', '位置3', '位置4', '位置5', '位置6'],
    'display': ['关', '开'],
    'sleep': ['关', '开'],
}
CONFIG_SCHEMA = climate.climate_schema(GreeYunyi2).extend({
    cv.Optional('poll_interval', default='1s'): cv.All(cv.positive_time_period_milliseconds, cv.Range(min=cv.TimePeriod(milliseconds=500), max=cv.TimePeriod(milliseconds=5000))),
    cv.Optional('aux_heat_write_enabled', default=False): cv.boolean,
    cv.Optional('report_current_temperature', default=False): cv.boolean,
    cv.Optional('packet_log', default=True): cv.boolean,
    cv.Optional('connection'): binary_sensor.binary_sensor_schema(device_class='connectivity'),
    **{cv.Optional(key): select.select_schema(GreeSelect) for key in OPTIONS},
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema('gree_yunyi2', baud_rate=4800, data_bits=8, parity='EVEN', stop_bits=1, require_tx=True, require_rx=True)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_poll_interval(config['poll_interval']))
    cg.add(var.set_aux_heat_write_enabled(config['aux_heat_write_enabled']))
    cg.add(var.set_report_current_temperature(config['report_current_temperature']))
    cg.add(var.set_packet_log(config['packet_log']))
    for kind, (key, options) in enumerate(OPTIONS.items()):
        if key in config:
            entity = await select.new_select(config[key], options=options)
            cg.add(entity.set_parent(var))
            cg.add(entity.set_kind(kind))
            cg.add(var.set_select(kind, entity))
    if 'connection' in config:
        entity = await binary_sensor.new_binary_sensor(config['connection'])
        cg.add(var.set_connection(entity))
