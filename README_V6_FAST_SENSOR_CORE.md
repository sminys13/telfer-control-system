# V6 Sensor Core FAST

Новая быстрая версия датчикового ядра без блокирующих `delay()` в основном цикле.

## Окружения PlatformIO

- `mega_v6_sensor_core` — безопасная медленная версия, которую уже проверили: ZERO/SAVE/LOAD/RESET работают.
- `mega_v6_sensor_core_fast` — новая быстрая версия для ускорения отображения и дальнейшей доводки.

## Что делает fast-версия

- X1/X2/Z1/Z2 остаются рабочими координатами.
- Используется неблокирующий single-shot опрос.
- Команды DWIN сохранены:
  - `0021` ZERO X1
  - `0022` ZERO X2
  - `0023` ZERO Z1
  - `0024` ZERO Z2
  - `0030` SAVE
  - `0031` LOAD
  - `0032` RESET
- Настройки читаются/пишутся в EEPROM тем же `settings_v6`.

## Запуск

```bash
pio run -e mega_v6_sensor_core_fast
pio run -e mega_v6_sensor_core_fast -t upload
```

Если fast-версия ведёт себя нестабильно, можно сразу вернуться к проверенной:

```bash
pio run -e mega_v6_sensor_core
pio run -e mega_v6_sensor_core -t upload
```

## Настройка скорости

В `include/config_v6_bringup.h`:

```cpp
FAST_SENSOR_PERIOD_MS = 120;
FAST_SENSOR_PHASE_MS = 30;
FAST_SENSOR_RESPONSE_TIMEOUT_MS = 260;
FAST_DWIN_UPDATE_MS = 80;
```

Если всё стабильно, можно уменьшать `FAST_SENSOR_PERIOD_MS` до 100, потом 90.
Если начинаются пропуски, вернуть 120–150.


## FAST CONTINUOUS STREAM UPDATE

Эта версия драйвера `fast_laser_sensor` больше не использует одиночные измерения.
Алгоритм канала:

1. `LASER ON`
2. `CONTINUOUS`
3. Дальше контроллер только читает входящий поток UART через SC16IS752 и парсит кадры `80 06 82/83 ...`.
4. Если поток пропал дольше `FAST_CONTINUOUS_RESTART_MS`, канал мягко перезапускает `LASER ON -> CONTINUOUS`.

Важно: координаты X1/X2/Z1/Z2 больше не сбрасываются в 0 при stale. Последнее валидное значение остаётся на экране, а проблема показывается через `ERROR`.
