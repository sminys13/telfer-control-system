# V6 SYSTEM STEP3 — Manual jog safe stubs

Эта версия продолжает рабочую базу `v6-system-step2`.

## Главное

- Исправлен обработчик DWIN-команд: убран лишний `return`, который мог блокировать выполнение команд.
- Добавлен безопасный слой ручных команд движения.
- Физические выходы, реле и частотники пока НЕ управляются.
- Все ручные команды пока только логируются в Serial Monitor.

## Сохранено

- Датчики: continuous, range 10m, resolution 1mm, freq 20Hz.
- Без READ_CACHE, без auto-restart, без SINGLE.
- Координаты X1/X2/Z1/Z2 не сбрасываются в 0 при stale/lost.
- ZERO/SAVE/LOAD/RESET CAL работают как раньше.
- Режимы DWIN 0001..0006 работают как раньше.

## DWIN VP

Все кнопки пишут в:

```text
VP = 1100
Return Key Code
Data auto-uploading = ON
```

## Основные команды

```text
0001 Manual
0002 Auto
0003 Home
0004 Stop
0005 Settings
0006 Calibration
```

## Калибровка

```text
0021 ZERO X1
0022 ZERO X2
0023 ZERO Z1
0024 ZERO Z2
0030 SAVE
0031 LOAD
0032 RESET CAL
```

## Новые ручные команды-заглушки

```text
0101 H1 / X1 FWD
0102 H1 / X1 BWD
0103 H2 / X2 FWD
0104 H2 / X2 BWD
0105 H1+H2 FWD
0106 H1+H2 BWD

0107 V1 / Z1 UP
0108 V1 / Z1 DOWN
0109 V2 / Z2 UP
010A V2 / Z2 DOWN
010B V1+V2 UP
010C V1+V2 DOWN

010F JOG STOP
```

Команды движения выполняются только если текущий режим `MANUAL`.
В этой версии они только пишут в терминал, например:

```text
MANUAL JOG REQUEST axis=H1/X1 dir=POS/FWD/UP
```

## Проверка

1. Нажать `Manual` на DWIN, поле `MODE` должно стать `1`.
2. Нажать ручную кнопку `0101`, в терминале должно появиться сообщение о ручном движении.
3. Нажать `010F` или общий `STOP 0004`, в терминале должно появиться `MANUAL JOG STOP` или `MOTOR STOP ALL`.
4. Нажать ручную команду вне режима `MANUAL`: команда должна быть проигнорирована.

## Дальше

Следующий слой — реальный драйвер RS485/Modbus RTU для частотников. Команды из `MotorControlV6::requestManualMove()` будут заменены на реальные команды VFD.
