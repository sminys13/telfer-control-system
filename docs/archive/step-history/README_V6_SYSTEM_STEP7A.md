# V6 SYSTEM STEP7A — DWIN settings foundation

Рабочая среда PlatformIO остаётся:

```ini
mega_v6_sensor_core_fast
```

Этот шаг добавляет настраиваемую основу RS485/NE200 и не включает физическое управление приводами.

```cpp
VFD_RS485_ENABLED = false;
VFD_DRY_RUN       = true;
```

## Что сделано

- `SettingsV6` обновлён до версии 2.
- Добавлена автоматическая миграция EEPROM версии 1 с сохранением калибровки X1/X2/Z1/Z2.
- В EEPROM теперь хранятся:
  - скорость, чётность и стоп-биты RS485;
  - таймаут, повторы и интервалы опроса;
  - адреса H1/H2/V1/V2;
  - маска инверсии направлений;
  - watchdog ручного движения;
  - профили скорости четырёх приводов.
- Добавлены `APPLY / SAVE / LOAD / DEFAULTS`.
- Добавлена проверка значений и повторяющихся Modbus-адресов.
- Настройки применяются только при остановленном движении в режимах STOP/SETTINGS/SERVICE/CALIBRATION.
- Любой переход из активного движения в другой режим сначала вызывает `stopAll()`.
- После dry-run STOP статус VFD становится `STOPPED = 4`, а не остаётся `DRY_RUN = 2`.
- DWIN теперь принимает не только `VP_CMD`, но и auto-upload от редактируемых полей.
- GitHub Actions переведён на активную среду `mega_v6_sensor_core_fast`.

## Настройки по умолчанию

```text
Baud:              9600
Parity:            EVEN
Stop bits:         1
Response timeout:  150 ms
Retries:           1
Inter-request:     10 ms
Online poll:       100 ms
Offline poll:      1000 ms
Addresses:         H1=1, H2=2, V1=3, V2=4
Manual watchdog:   2500 ms
```

Профили:

```text
H1/H2: manual 20%, max 80%, slow 10%, slowdown 500 mm, tolerance 10 mm
V1/V2: manual 20%, max 60%, slow 10%, slowdown 400 mm, tolerance 8 mm
```

## DWIN: редактируемые VP

Каждое поле должно быть `Int` и иметь включённый `Data auto-uploading`.

```text
1200  baud code: 0=4800, 1=9600, 2=19200, 3=38400, 4=57600, 5=115200
1202  parity: 0=None, 1=Even, 2=Odd
1204  stop bits: 1 или 2
1206  response timeout, ms
1208  retries, 0..5
120A  inter-request delay, ms
120C  online poll period, ms
120E  offline poll period, ms

1210  H1 address
1212  H2 address
1214  V1 address
1216  V2 address
1218  invert-direction mask, bits 0..3
1220  manual jog watchdog, ms
```

Профили приводов:

```text
H1 base 1230
H2 base 1240
V1 base 1250
V2 base 1260

base+0  manual speed, %
base+2  max speed, %
base+4  slow speed, %
base+6  slowdown distance, mm
base+8  stop tolerance, mm
```

## DWIN: статус редактора

```text
1280  settings state
1282  validation error mask
1284  dirty: 0/1
1286  EEPROM settings version
1288  test drive: 1=H1, 2=H2, 3=V1, 4=V2, 5=ALL
```

`VP_SETTINGS_STATE = 1280`:

```text
0 IDLE
1 DIRTY
2 APPLIED
3 SAVED
4 LOADED
5 DEFAULTS loaded into editor
6 REJECTED
7 TEST DRY-RUN
```

`VP_SETTINGS_ERROR = 1282` — битовая маска:

```text
0001 baud
0002 parity
0004 stop bits
0008 response timeout
0010 retries
0020 intervals
0040 address out of range
0080 duplicate addresses
0100 manual watchdog
0200 drive profile
0400 sensor settings
0800 EEPROM record
8000 unsafe mode / movement active
```

## DWIN: кнопки

Все кнопки: `Return Key Code`, `VP=1100`, `Data auto-uploading=ON`.

```text
0200 APPLY
0201 SAVE
0202 LOAD
0203 DEFAULTS
0204 TEST H1
0205 TEST H2
0206 TEST V1
0207 TEST V2
0208 TEST ALL
```

Тесты пока только печатают план безопасного READ в терминал. Никакого кадра в RS485 не отправляется.

## Поведение кнопок

- `APPLY` — проверяет значения и применяет их только в RAM.
- `SAVE` — проверяет, применяет и сохраняет в EEPROM.
- `LOAD` — загружает сохранённые значения и применяет их.
- `DEFAULTS` — заполняет редактор безопасными значениями, но не применяет и не сохраняет автоматически.
- `TEST` — dry-run диагностика выбранного адреса.

## Проверка без страницы DWIN

После прошивки существующий функционал продолжает работать. В терминале при запуске ожидается:

```text
Settings: EEPROM v1 migrated to v2; calibration preserved
```

или при последующих стартах:

```text
Settings: loaded from EEPROM v2
```

При ручном движении watchdog использует значение из EEPROM. После остановки:

```text
motor=20 STOPPED vfd=4
```

## Следующий шаг

Step7B подключит уже существующие `modbus.cpp`, `motors.cpp`, `utils.cpp` и сформирует реальные NE200 Modbus RTU кадры в dry-run. Физическую передачу по RS485 пока также оставим выключенной.
