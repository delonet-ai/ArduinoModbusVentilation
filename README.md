# ArduinoModbusVentilation

Проект управляет сервоприводами вентиляционных заслонок через Arduino Uno/Nano и Wiren Board по Modbus RTU.

Внешний интерфейс работает только с процентами открытия: `0` означает закрыто, `100` означает полностью открыто. Перевод процента в команду сервопривода выполняется внутри прошивки Arduino.

## Modbus

- Протокол: Modbus RTU slave.
- Slave ID: `128`.
- Скорость и формат порта WB: `9600 8N1`.
- RS-485 DE/RE pin: `D2`.
- Управляющие регистры: holding registers.

| Регистр | Канал | Значение |
| --- | --- | --- |
| `0` | `Kitchen_out` | `0..100%` |
| `1` | `Cabinet_out` | `0..100%` |
| `2` | `Bedroom_out` | `0..100%` |
| `3` | `Living_out` | `0..100%` |
| `4` | `Kid_out` | `0..100%` |
| `5` | `Kitchen_in` | `0..100%` |
| `6` | `Cabinet_in` | `0..100%` |
| `7` | `Bedroom_in` | `0..100%` |
| `8` | `Living_in` | `0..100%` |
| `9` | `Kid_in` | `0..100%` |
| `10` | `Test Mode` | `0` off, `1` on |
| `11..20` | Closed endpoints | `0..100%`, device settings only |
| `21..30` | Open endpoints | `0..100%`, device settings only |

Endpoint settings are calibration values. They do not change the user-facing `0..100%` controls:

- user control `0%` maps to the channel closed endpoint;
- user control `100%` maps to the channel open endpoint;
- defaults are `0` and `100`;
- values are stored in Arduino EEPROM.

## Servo Test Mode

`Test Mode` - временный режим проверки сервоприводов из UI Wiren Board.

Когда регистр `10` включён, Arduino игнорирует рабочие значения регистров `0..9` и по очереди двигает каждую серву по циклу `0% -> 100% -> 0%`. Один полный цикл одной сервы занимает около 5 секунд. Тестовые движения не сохраняются в EEPROM.

Когда `Test Mode` выключается, Arduino возвращается к обычному управлению по текущим регистрам `0..9`.

Чтобы потом удалить тестовый режим, уберите:

- регистр `TEST_MODE_REGISTER` и блок `SERVO_TEST_MODE` из скетча;
- канал `Test Mode` из `wirenboard/templates/config-arduino-modbus-servo.json`;
- описание тестового режима из README.

## Arduino Pins

Сервоприводы подключаются к пинам:

`D8`, `D9`, `D10`, `D11`, `D12`, `A0`, `A1`, `A2`, `A3`, `A4`.

RS-485 модуль:

- `DI`/`RO` к UART Arduino;
- `DE` и `RE` вместе к `D2`;
- общий GND между Arduino, RS-485 модулем и питанием серв.

Питание сервоприводов должно быть отдельным и рассчитанным на ток серв. Не питать все сервы от 5V Arduino.

## Build And Upload

Установите `arduino-cli`, AVR core и библиотеки:

```powershell
arduino-cli core install arduino:avr
arduino-cli lib install ModbusRtu
arduino-cli lib install ServoSmooth
```

Если `ModbusRtu` не находится в Arduino Library Manager, установите библиотеку вручную:

```powershell
git clone https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino.git "$env:USERPROFILE\Documents\Arduino\libraries\Modbus-Master-Slave-for-Arduino"
```

Сборка для Arduino Uno:

```powershell
arduino-cli compile --fqbn arduino:avr:uno .\ArduinoModbusVentilation
```

Прошивка через вероятный порт `COM6`:

```powershell
arduino-cli upload -p COM6 --fqbn arduino:avr:uno .\ArduinoModbusVentilation
```

## Wiren Board

Шаблон устройства находится в `wirenboard/templates/config-arduino-modbus-servo.json`.

Установка шаблона на Wiren Board:

```bash
scp wirenboard/templates/config-arduino-modbus-servo.json root@192.168.0.222:/usr/share/wb-mqtt-serial/templates/config-arduino-modbus-servo.json
ssh root@192.168.0.222 "systemctl restart wb-mqtt-serial && journalctl -u wb-mqtt-serial -n 80 --no-pager"
```

На Windows можно использовать готовый скрипт:

```powershell
.\scripts\deploy-wb-template.ps1
```

Параметры устройства в `/etc/wb-mqtt-serial.conf`:

- port: тот RS-485 порт, куда подключена Arduino, например `/dev/ttyRS485-2`;
- `baud_rate`: `9600`;
- `data_bits`: `8`;
- `parity`: `N`;
- `stop_bits`: `1`;
- `device_type`: `arduino_modbus_servo`;
- `slave_id`: `128`.

После установки добавьте или обновите устройство Modbus RTU в UI Wiren Board, выбрав шаблон `VentilationArduinoServo`. Если устройство уже было заведено старым шаблоном, обновите `device_type` на `arduino_modbus_servo` и перезапустите `wb-mqtt-serial`.

Если каналы в UI стали красными после исправленной связи, проверьте retained MQTT ошибки:

```bash
mosquitto_sub -t '/devices/arduino_modbus_slave_128/controls/+/meta/error' -v
```

Старые retained ошибки можно очистить пустой retained публикацией в соответствующие `.../meta/error` топики.

На Windows для этого есть скрипт:

```powershell
.\scripts\clear-wb-control-errors.ps1
```
