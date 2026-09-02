# V6 SYSTEM STEP9B — AUTO timing fix + quieter USB console

## Что исправлено после реального теста Step9A

На Mega команда `auto` из USB-консоли могла стартовать AutoRunner уже после того, как
главный `loop()` сохранил локальное значение `now`. В том же проходе `service()` получал
timestamp на несколько миллисекунд старше `_phaseStartMs`. Беззнаковое вычитание
в `checkMovementTimeout()` выглядело как почти полный период `millis()` (~49 суток),
поэтому сразу возникал `AUTO FAULT code=5 movement phase timeout`.

Step9B исправляет это двумя независимыми мерами:

1. timestamp главного цикла снимается после `serviceUsbConsole()`;
2. AutoRunner игнорирует маленький (<=1000 ms) обратный скачок timestamp как устаревший
   sample, при этом нормальное переполнение `millis()` продолжает корректно работать.

## USB-лог стал тише

По умолчанию периодическая строка состояния выводится раз в 2 секунды.

Команды:

- `log quiet` — убрать периодические строки; события, фазы и ошибки остаются;
- `log normal` — статус раз в 2 секунды;
- `log verbose` — старый режим раз в 0.5 секунды.

## Проверка на столе

Собрать и залить среду:

`mega_v6_sensor_core_fast`

Рекомендуемый тест:

```
log quiet
clear
estop off
limits off
sim demo
auto
```

Ожидаемое начало:

```
AUTO SIMULATION ON
RAM-only SIM-DEMO loaded; it cannot be saved to EEPROM
MotorControlV6 mode hook: AUTO
Auto mode selected. AutoRunnerV6 owns the program sequence.
AUTO PHASE -> 1 PREP_TRAVEL
AUTO START program=SIM-DEMO mode=SIMULATION
AUTO PHASE -> 2 MOVE_ZONE_H
...
```

Финал должен быть `AUTO PROGRAM DONE` / `DONE`, без `AUTO FAULT code=5`.

Для наблюдения координат во время симуляции можно после старта ввести `log verbose`.

## Что не менялось

- физический MAX485 в `mega_v6_sensor_core_fast` по-прежнему выключен;
- READ-ONLY / FIELD MANUAL / FIELD AUTO сборки сохранены;
- E-STOP и концевики сохранены;
- структура 4 программ / 10 зон сохранена;
- алгоритм AUTO/HOME Step9A не менялся, исправлено только время и удобство консоли;
- один физический лазер по-прежнему достаточен для стенда в SIM, но реальный AUTO требует X1/X2/Z1/Z2.
