# Telfer v6 — step 3 bring-up

This package extends the confirmed step1/step2 baseline:

- DWIN on `Serial2`
- `SC16IS752 #1` for `X1` and `X2`
- `SC16IS752 #2` for `Z1`
- single-shot proven baseline kept for all channels

## Channels

- `SC16 #1 CH_A` -> `X1`
- `SC16 #1 CH_B` -> `X2`
- `SC16 #2 CH_A` -> `Z1`
- `SC16 #2 CH_B` reserved for `Z2`

## Wiring for SC16 #2

- `VCC -> 3.3V`
- `GND -> GND`
- `RESET -> 3.3V`
- `I2C/SPI -> GND`
- `SDA/VSS -> GND`
- `A0/CS -> D8`
- `A1/SI -> D51`
- `NC/SO -> D50`
- `SCL/SCLK -> D52`

## Important note

For channels driven by `SC16IS752`, keep the local cabinet-side `TTL↔RS422` module on `3.3V`.
This was verified on the user's hardware.

## DWIN expected view

- `X1` live
- `X2` live
- `Z1` live
- `Z2` stays `0` for now
