# V6 SYSTEM STEP6 — VFD / RS485 dry-run layer

Этот шаг ничего физически не включает. Он только готовит слой управления будущими частотниками по RS485.

## Что добавлено

1. `include/vfd_driver_v6.h`
2. `src/vfd_driver_v6.cpp`
3. Подключение `VfdDriverV6` внутрь `MotorControlV6`
4. Новый DWIN VP:

```text
VP_VFD_STATUS = 0x1016
```

На экран это поле добавлять необязательно. Если добавить, то значения такие:

```text
0 — VFD disabled
1 — VFD ready
2 — dry-run / команды только логируются
3 — moving
4 — stopped
5 — blocked
```

## Безопасность

В `include/config_v6_bringup.h` сейчас:

```cpp
static constexpr bool VFD_RS485_ENABLED = false;
static constexpr bool VFD_DRY_RUN       = true;
```

Поэтому реальные команды на RS485 не отправляются. В терминал только пишется, что бы было отправлено в будущем.

## Проверка

1. Перейти в ручной режим: `0001`.
2. Нажать ручную команду, например `0101`.
3. В терминале должно появиться:

```text
MANUAL JOG REQUEST ...
VFD DRY-RUN START motor_state=1; real Modbus command is not enabled yet
```

4. Нажать `010F`.
5. В терминале должно появиться:

```text
MANUAL JOG STOP reason=DWIN jog stop
VFD DRY-RUN STOP ALL reason=DWIN jog stop; real Modbus stop is not enabled yet
```

## Важно

Этот слой специально не содержит реальных регистров частотников. Их надо добавить позже, когда будет известна точная модель ПЧ и карта Modbus-регистров.

## TODO, чтобы не забыть

1. `MANUAL_JOG_TIMEOUT_MS` сейчас задан в конфиге. Позже перенести в `SettingsV6`, сохранять в EEPROM и сделать настройку с DWIN.
2. После появления частотников добавить точную карту Modbus-регистров:
   - адрес ПЧ;
   - регистр частоты;
   - регистр RUN/STOP;
   - направление;
   - масштаб частоты;
   - чтение статуса/аварии.
3. Не включать `VFD_RS485_ENABLED=true`, пока не проверены схема MAX485, DE/RE, A/B и регистры ПЧ.
