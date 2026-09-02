# V6 Continuous 10Hz + READ_CACHE experiment

Режим для проверки после no-restart:

1. LASER ON
2. SET RANGE 10m
3. SET RESOLUTION 1mm
4. SET FREQUENCY 10Hz
5. CONTINUOUS
6. Без автоматического рестарта
7. Периодически отправляется READ_CACHE, чтобы забрать последнее кэшированное значение, если пассивный поток залип при движении.

Ключевые параметры в `include/config_v6_bringup.h`:

```cpp
FAST_SENSOR_PERIOD_MS = 80;
FAST_SENSOR_PHASE_MS = 20;
FAST_CONTINUOUS_USE_READ_CACHE = true;
```

Если станет хуже — вернуть `FAST_CONTINUOUS_USE_READ_CACHE = false` или увеличить `FAST_SENSOR_PERIOD_MS` до 100/120.
