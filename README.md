# ESPHome компонент для подключения счетчиков электроэнергии Энергомера CE102M CE301 по RS-485 (ГОСТ IEC 61107-2011)
Компонент для считывания данных с электросчетчиков, поддерживающих протокол МЭК/IEC 61107.
Большинство счетчиков имеют свои особенности, требуется тестирование конкретных моделей.
Пока в разработке поддержка Энергомера СЕ102М

На данный момент реализовано:
- поддержка только одной скорости, без изначального подключения по 300 бод 
- параметры считываются индивидуально
- пока поддерживается считывание только одиночного параметра из ответа: например, `VOLTA(228.93)` - берется 228.93 из скобок
- `sensor` - для параметров числовых (float)
- `text_sensor` - для любых других

Названия функций для запроса берем из документации на счетчик.
Например, [Руководство по эксплуатации CE102M](http://sp.energomera.ru/documentations/product/ce102m_re_full.pdf), приложение Д.

Пример конфига для CE102M ниже.

Планы:
- подключение на 300 бод и выбор скорости согласно ответу от счетчика
- режим readout
- поддержка считывания нескольких параметров из ответа

## Подключение
Компонент предполагает использование ESP8266 или ESP32 с RS-485 трансивером (на 3.3 вольта!).
Подключение счетчика к трансиверу по одной витой паре.
Подключение микроконтроллера к трансиверу трехпроводное: 
``RO`` - прием, 
``DI`` - передача, 
``DE + R̅E̅``  - контроль линии для передачи данных.
```
+-------+                    +-------------+              +----------------+
|       |                    |             |              |                |
|  MCU  | RX <----------< RO | RS485<->TTL | A <------> A | Электросчетчик |
|       |             ,-> R̅E̅ |             |              |                |
| ESPxx | FLOW >-----+       |   module    | B <------> B |                |
|       |             `-> DE |    3.3v!    |              |                |
|       | TX >----------> DI |             |              |                |
+-------+                    +-------------+              +----------------+
```
### Особенности
0. Некоторые модули rs485 автоматически переключают режим передачи/приема, тогда подключение `re/de` и настройка `flow_control_pin` к конфиге не требуется.

1. UART0 использовать нельзя - ESPHOME его использует для логирования.

2. Для esp8266 в ESPHOME используется SoftwareSerial и можно использовать любые пины для RX/TX. 

3. Для esp32 желательно использовать UART2: RX=16, TX=17, но должен работать и любой другой.

4. Параметры подключения к счетчику: 9600 7E1.

5. Компонент расчитан на работу с одним электросчетчиком и использует широковещательные пакеты.


# EspHome component for IEC61107 electricity meteres
EspHome Electricity meters IEC61107 - Energomera CE102M, CE301

Basic support of iec61107 meters
- fixed 9600 baud
- reading individual registers (no auto readout)
- only replies in form `PARAMETER(data)` are supported.
- use `sensor` if `data` is numeric
- use `text_sensor` in other cases


# CE102M yaml config
```
esphome:
  name: energomera-ce102

esp32:
  board: esp32dev
  framework:
    type: arduino

logger:
  level: DEBUG

external_components:
  - source: github://latonita/esphome-iec61107-meter
    refresh: 10s
    components: [iec61107]

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  reboot_timeout: 5min
  power_save_mode: NONE

api:
  password: !secret api_password

ota:
  password: !secret ota_password

uart:
    rx_pin: GPIO16
    tx_pin: GPIO17
    baud_rate: 9600
    data_bits: 7
    parity: EVEN
    stop_bits: 1

iec61107:
  id: ce102
  update_interval: 30s
#  receive_timeout: 500ms
#  flow_control_pin: GPIO32
#  readout_enabled: true

sensor:
  - platform: iec61107
    request: ET0PE(02)
    name: Consumed energy T1
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing

  - platform: iec61107
    request: ET0PE(03)
    name: Consumed energy T2
    unit_of_measurement: kWh
    accuracy_decimals: 3
    device_class: energy
    state_class: total_increasing

  - platform: iec61107
    name: Current L1
    request: CURRE()
    unit_of_measurement: A
    accuracy_decimals: 2
    device_class: current
    state_class: measurement

  - platform: iec61107
    name: Voltage L1
    request: VOLTA()
    unit_of_measurement: V
    accuracy_decimals: 1
    device_class: voltage
    state_class: measurement

  - platform: iec61107
    name: Frequency L1
    request: FREQU()
    unit_of_measurement: Hz
    accuracy_decimals: 2
    device_class: frequency
    state_class: measurement

  - platform: iec61107
    name: Power factor
    request: COS_f()
    unit_of_measurement: "%"
    accuracy_decimals: 2
    device_class: power_factor
    state_class: measurement

  - platform: iec61107
    name: Power
    request: POWEP()
    unit_of_measurement: kW
    accuracy_decimals: 3
    device_class: power
    state_class: measurement
  
text_sensor:
  - platform: iec61107
    name: Serial number
    request: SNUMB()

  - platform: iec61107
    name: Time
    request: TIME_()

  - platform: iec61107
    name: Date
    request: DATE_()

  - platform: iec61107
    name: Info
    request: VINFO()

  - platform: iec61107
    name: Model
    request: MODEL()
```
