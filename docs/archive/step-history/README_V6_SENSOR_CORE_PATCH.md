# V6 Sensor Core Patch

This project keeps the original files in place. Nothing was deleted or moved.

Use this environment in PlatformIO:

```bash
pio run -e mega_v6_sensor_core
pio run -e mega_v6_sensor_core -t upload
```

## DWIN buttons

All buttons must be configured as:

- Return Key Code
- VP = `0x1100`
- Data auto-uploading = ON

Command values:

- `21` — ZERO X1
- `22` — ZERO X2
- `23` — ZERO Z1
- `24` — ZERO Z2
- `30` — SAVE settings to EEPROM
- `31` — LOAD settings from EEPROM into RAM
- `32` — RESET calibration in RAM

## Meaning of LOAD and RESET

LOAD:
- rereads settings from EEPROM without rebooting;
- useful if you changed calibration in RAM and want to return to the last saved values;
- if EEPROM data is invalid, defaults are restored in RAM.

RESET:
- clears calibration offsets in RAM;
- does NOT save automatically;
- press SAVE after RESET if you want reset calibration to remain after reboot.

## DWIN fields

- `0x1000` — X1 coordinate
- `0x1002` — X2 coordinate
- `0x1004` — Z1 coordinate
- `0x1006` — Z2 coordinate
- `0x1010` — MODE, should show `20`
- `0x1012` — ERROR bitmask:
  - bit 0 = X1 invalid/stale
  - bit 1 = X2 invalid/stale
  - bit 2 = Z1 invalid/stale
  - bit 3 = Z2 invalid/stale

## Notes

The current sensor reading layer uses the reliable blocking single-shot driver.
It is intended for calibration and functional bring-up. Fast motion optimization remains a separate step.
