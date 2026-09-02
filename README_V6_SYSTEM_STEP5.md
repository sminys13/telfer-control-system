# V6 SYSTEM STEP5 — Manual jog watchdog

Этот шаг продолжает безопасный ручной режим. Физические выходы и частотники по-прежнему не управляются.

## Что добавлено

1. `MANUAL_JOG_TIMEOUT_ENABLED` и `MANUAL_JOG_TIMEOUT_MS` в `include/config_v6_bringup.h`.
2. Если ручная команда движения активна дольше заданного времени без новой команды или STOP, `MotorControlV6` переводит состояние в `STOPPED`.
3. `VP_MOTOR_STATE = 0x1014` автоматически обновляется после watchdog-stop.

## Зачем это нужно

Когда подключим реальные частотники, watchdog защитит от ситуации, когда DWIN/линия связи отправила команду движения, а команда остановки не пришла.

## Проверка

1. Нажать `0001` — Manual.
2. Нажать `0101` — motor_state станет `1`.
3. Не нажимать STOP. Через `MANUAL_JOG_TIMEOUT_MS` motor_state должен стать `20`.
4. Нажать `010F` — motor_state сразу станет `20`.

## Настройка

В `config_v6_bringup.h`:

```cpp
static constexpr bool MANUAL_JOG_TIMEOUT_ENABLED = true;
static constexpr uint16_t MANUAL_JOG_TIMEOUT_MS = 2500;
```

Если для тестов нужно оставить состояние движения защёлкнутым до STOP, временно поставь `false`.
