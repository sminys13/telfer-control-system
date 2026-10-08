# V6 SYSTEM STEP9F — полевая диагностика HE200 и лазерных датчиков

## Назначение

Step9F подготавливает безопасное полевое испытание:

- промышленного изолированного Waveshare TTL TO RS485 (B);
- четырёх HE200 в режиме **только чтение**;
- четырёх UART-лазеров через два SC16IS752;
- полных команд конфигурации лазеров с префиксом `FA`;
- подтверждений ACK/NACK и статистики ошибок;
- сервисного браузерного приложения вместо ручного набора команд.

Физические команды движения HE200 по RS485 в Step9F **запрещены при компиляции**. Тельферы во время испытаний перемещаются только существующим физическим пультом.

## Рабочая сборка для объекта

```text
mega_v6_he200_waveshare_readonly
```

Сборки проекта:

| Среда | Назначение |
|---|---|
| `mega_v6_sensor_core_fast` | Стенд без физического RS485 ПЧ |
| `mega_v6_he200_waveshare_readonly` | Новый Waveshare, автоматическое направление, HE200 только чтение |
| `mega_v6_he200_max485_readonly` | Старый MAX485, ручной DE/RE на pin 6, только резервная диагностика |

COM-порт больше не зашит в `platformio.ini`.

## Подключение Waveshare к Mega

```text
Arduino Mega 2560       Waveshare TTL TO RS485 (B)
--------------------------------------------------
5V                  ->  VCC
GND                 ->  GND  (TTL-сторона)
pin 18, TX1         ->  RXD
pin 19, RX1         <-  TXD

Waveshare A+        ->  HE200 A+
Waveshare B-        ->  HE200 A-
```

- pin 6 Mega не подключать;
- `SGND` в первом испытании не подключать;
- переключатель `120R` для короткого метрового подключения поставить `OFF`;
- существующее клеммное управление HE200 не менять.

## HE200

Сборка Step9F при каждом запуске автоматически применяет только в RAM:

```text
9600 8-N-1
H1 = address 1
H2 = address 2
V1 = address 3
V2 = address 4
timeout = 250 ms
inter-request = 20 ms
```

EEPROM автоматически не перезаписывается.

Проверки:

```text
test h1
test h2
test v1
test v2
test all
```

Структурированная строка для приложения:

```text
@HE200 name=H1 addr=1 online=1 run001=... set001=... bus01=... outV=... outI001=... di=... fault=... state=... error=0
```

При таймауте:

```text
@HE200 name=H1 addr=1 online=0 error=1
```

## Лазерный протокол

Step9F отправляет полную последовательность:

```text
80 04 02 7A        shutdown
80 06 05 01 74     laser ON
FA 04 09 0A EF     range 10 m
FA 04 0C 01 F5     resolution 1 mm
FA 04 0A xx CS     frequency 5/10/20 Hz
80 06 03 77        continuous measurement
```

Ожидаемые ACK:

```text
80 04 82 FA        shutdown
80 06 85 01 F4     laser ON
FA 04 89 79        range
FA 04 8C 76        resolution
FA 04 8A 78        frequency
```

`ack=0x3F` означает, что получены пять ACK и затем хотя бы один потоковый кадр.

## Сервисные команды

```text
laser status
laser reset
laser reinit
laser config all 5
laser config all 10
laser config all 20
laser config x1 10
laser config x2 10
laser config z1 10
laser config z2 10
report
```

Поле `laser status`:

```text
@LASER name=X1 hw=1 valid=1 mm=8806 age=... byteAge=...
freq=10 cfg=0 ack=0x3F missing=0x0 nack=0x0 rate10=100
good=... stream=... sensorErr=... errCode=... crc=...
malformed=... range=... discard=... swOv=...
uartOE=... uartPE=... uartFE=... uartBI=... uartFIFO=...
```

Интерпретация:

- `sensorErr` растёт, а `crc` и `uart*` равны нулю — датчик отвечает, но не получает надёжное отражение;
- `crc` или `uart*` растут — проверять линию датчика, питание, преобразователь и SC16;
- `byteAge` растёт и ничего больше не меняется — с канала перестали приходить байты;
- `missing` — ожидаемый ACK не был получен до таймаута;
- `nack` — датчик явно отверг команду;
- `rate10=100` означает 10,0 кадра/с.

## Сервисное приложение

Путь:

```text
tools/telfer_service_console/start_console.bat
```

Приложение:

- подключается к выбранному COM-порту на 115200;
- отправляет команды кнопками;
- показывает таблицы четырёх лазеров и четырёх HE200;
- сохраняет журнал;
- проводит отдельные временные сеансы 5/10/20 Гц;
- не содержит команд движения.

Перед подключением приложения закрыть PlatformIO Monitor.

## Проверки, выполненные при подготовке

- команды и ACK лазеров проверены по контрольной сумме;
- парсер проверен на кадре `008.806`, кадре `ERR--16`, ACK частоты и повреждённой checksum;
- все активные исходники трёх профилей прошли host-синтаксическую компиляцию `-Wall -Wextra -Werror`;
- JavaScript сервисной консоли прошёл `node --check`.

Полная AVR/PlatformIO-сборка должна быть выполнена на компьютере пользователя перед поездкой, поскольку в среде подготовки не установлен toolchain PlatformIO/AVR.
