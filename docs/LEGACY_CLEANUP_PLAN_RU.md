# План обращения с legacy-кодом

## Решение

Legacy-код не должен бесконечно лежать в рабочей ветке, но удалять его перед полевым испытанием Step9H не следует.

Профессиональная схема:

1. сохранить историческое состояние отдельным Git-тегом и архивной веткой;
2. подтвердить Step9H на объекте;
3. удалить гарантированно неиспользуемые файлы из активной ветки;
4. оставить историю в Git, а не копии мёртвого кода рядом с рабочим кодом.

Git сам является архивом: после коммита удалённые файлы остаются доступными в истории и в теге.

## Что пока оставить

Следующие файлы входят в активные Step9H-сборки и удалять их нельзя:

```text
main_v6_fast.cpp
dwin_link.cpp
fast_laser_sensor.cpp
sc16is752.cpp
settings_v6.cpp
system_state.cpp
motor_control_v6.cpp
vfd_driver_v6.cpp
modbus.cpp
motors.cpp
utils.cpp
safety_v6.cpp
program_v6.cpp
auto_runner_v6.cpp
```

Особенно важно: `modbus.cpp`, `motors.cpp` и `utils.cpp` выглядят историческими, но реально компилируются в V6. Их надо сначала разделить/переименовать, а не удалять.

## Кандидаты на удаление после полевого подтверждения

Эти старые подсистемы не входят ни в один из трёх поддерживаемых Step9H build-профилей:

```text
src/app.cpp
src/ui.cpp
src/keypad.cpp
src/laser_sensor.cpp
src/sensors.cpp
src/storage.cpp
src/main.cpp

include/app.h
include/ui.h
include/keypad.h
include/laser_sensor.h
include/sensors.h
include/storage.h
include/config.h
```

Перед удалением выполнить поиск ссылок и сборку всех поддерживаемых сред.

## Документация и патчи

Старые README и промежуточные `.patch` не должны засорять корень проекта. Их можно хранить в:

```text
docs/archive/step-history/
```

После стабилизации достаточно оставить в корне:

```text
README.md
README_V6_SYSTEM_STEP9H_RU.md
FIELD_TEST_STEP9H_RU.md
FIELD_QUICK_CARD_STEP9H_RU.md
WIRING.md
```

## Git-последовательность после испытаний

```bash
git checkout v6_ttl-422
git pull
git tag -a archive-pre-step9h-cleanup -m "История проекта до удаления legacy"
git branch archive/legacy-lcd-v5
```

Затем создать отдельную ветку очистки:

```bash
git switch -c cleanup/remove-unused-legacy
```

Удалять legacy одним отдельным коммитом, не смешивая с изменениями логики:

```bash
git commit -m "chore: archive and remove unused legacy LCD stack"
```

## Критерии готовности к удалению

- Step9H собирается во всех трёх средах;
- четыре лазера прошли полевой тест;
- Waveshare читает четыре HE200;
- рабочий код не включает legacy-заголовки;
- создан архивный тег;
- новая документация описывает только фактическое оборудование.
