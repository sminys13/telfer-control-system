# V6 System Step8A — field safety + persistent DWIN diagnostics

Основа: проверенный `v6-system-step7b-ne200-modbus-dry-run`.

## Зачем этот шаг

Перед подключением AUTO/HOME и тем более перед физическим MAX485 в активную V6-сборку возвращена аппаратная защита старого проекта:

- E-STOP: pin 2;
- H1 LEFT/RIGHT: pins 36/37;
- H2 LEFT/RIGHT: pins 38/39.

Физический RS485 всё ещё **выключен**. Это принципиально: Step8A проверяется только по DWIN и Serial Monitor.

## Режим безопасности

По умолчанию:

```cpp
SAFETY_BENCH_MODE = true;
VFD_RS485_ENABLED = false;
VFD_DRY_RUN = true;
```

В bench mode вход считается активным при LOW (удобно для NO-кнопки на GND или незадействованных входов с `INPUT_PULLUP`).

Для реального шкафа с рекомендуемой NC-цепью нужно после проверки проводки поставить:

```cpp
SAFETY_BENCH_MODE = false;
```

При этом healthy NC-to-GND = LOW, а нажатие/обрыв = HIGH = авария.

Добавлен `static_assert`: физический `VFD_RS485_ENABLED=true` нельзя собрать, пока `SAFETY_BENCH_MODE=true`.

## Поведение E-STOP

- активный E-STOP немедленно ставит `STOP ALL`;
- режим переводится в STOP даже если E-STOP сработал в покое;
- при старте с уже активной цепью система сразу остаётся в STOP;
- E-STOP защёлкивается программно;
- отпускание кнопки само по себе движение не разрешает;
- пока E-STOP активен или защёлкнут, переход в MANUAL/AUTO/HOME блокируется;
- сброс защёлки: `0x0044 CLEAR STATUS` или отдельная `0x0047 SAFETY CLEAR`;
- сброс при всё ещё активном E-STOP отвергается.

## Поведение концевиков

Концевики не запрещают движение *от* сработавшего концевика. Они блокируют только движение *в него*:

- H1 FWD/right блокируется H1 RIGHT;
- H1 BWD/left блокируется H1 LEFT;
- H2 FWD/right блокируется H2 RIGHT;
- H2 BWD/left блокируется H2 LEFT;
- для H1+H2 достаточно соответствующего концевика любой стороны, чтобы остановить пару.

Вертикальные концевики в старой аппаратной конфигурации отсутствовали, поэтому Step8A их не выдумывает.

## Новые DWIN VP для постоянных строк

Базовые VP `X1/X2/Z1/Z2`, `MODE`, `ERROR`, `MOTOR_STATE`, `VFD_STATUS`, `CMD` не менялись.

Добавлены:

```text
0x1020 PROGRAM_INDEX
0x1022 ZONE_CURRENT
0x1024 ZONE_TOTAL
0x1026 STEP_CURRENT
0x1028 STEP_TOTAL

0x1030 ELAPSED_H
0x1032 ELAPSED_M
0x1034 ELAPSED_S
0x1036 TOTAL_H
0x1038 TOTAL_M
0x103A TOTAL_S

0x1040 SENSOR_OK_COUNT
0x1042 SENSOR_TOTAL
0x1044 VFD_OK_COUNT
0x1046 VFD_TOTAL
0x1048 RS485_STATE       0=off, 1=physical, 2=dry-run
0x104A NETWORK_STATE     currently 0/planned
0x104C SAFETY_STATE      bit0=E-stop active, bit1=latched
0x104E LIMIT_STATE       bits H1L/H1R/H2L/H2R
0x1050 MODBUS_QUEUE_BUSY 0/1
```

Поля программы/зоны/шага/времени пока честно равны нулю. Никакие тестовые значения не подставляются. Их начнёт заполнять program runner в следующем шаге.

`SENSOR_OK_COUNT` считает канал online, если есть валидное удерживаемое значение не старше `staleTimeoutMs`. Поэтому STALE ещё считается online, LOST — нет.

`VFD_OK_COUNT` в dry-run будет 0/4, потому что это именно число реально ответивших ПЧ, а не число настроенных адресов.

## Новые сервисные команды

Все пишутся в `VP_CMD=0x1100`:

```text
0x0046 DIAG SNAPSHOT
0x0047 SAFETY CLEAR
```

`0x0046` печатает одной строкой состояние датчиков, ПЧ, RS485, E-STOP, концевиков и очереди Modbus.

## Что проверить

1. Сборка `mega_v6_sensor_core_fast`.
2. Стартовая строка должна содержать `SafetyV6: mode=BENCH ...`.
3. В обычном bench-подключении без нажатых входов: `saf=0 lim=0x0`.
4. Замкнуть E-STOP input pin 2 на GND: `SAFETY E-STOP ACTIVE`, ошибка получает бит `0x0200`, движение блокируется.
5. Отпустить: latch остаётся; jog должен быть BLOCKED.
6. Команда `0x0047`: latch сбрасывается, если E-STOP уже отпущен.
7. Замкнуть H1 RIGHT (pin 37) на GND: `lim=0x2`.
8. H1 FWD `0x0101` должен быть BLOCKED; H1 BWD `0x0102` должен формировать dry-run кадры.
9. Остальные manual-jog и STOP должны работать как Step7B.
10. Команда `0x0046` должна дать `DIAG sensors=.../4 vfd=0/4 rs485=DRY/DISABLED ...`.

## Следующий шаг

После этого прогона — Step8B: перенос технологического `AutoRunner` и HOME из старой `app.cpp` на четыре лазерных канала `X1/X2/Z1/Z2`, с отдельным EEPROM-блоком программ/зон и без включения физического RS485.
