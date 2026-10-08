# Telfer v6 — step 1 bring-up

This package is a safe first step for the new architecture:

- DWIN on `Serial2` through `MAX3232`
- one `SC16IS752`
- one laser on `SC16 channel A`
- one local and one remote `TTL↔RS422`

## What this package does

- initializes DWIN
- initializes `SC16IS752`
- initializes one laser channel
- reads one laser in a stable single-shot mode
- writes the measured value to DWIN field `X1` (`VP 0x1000`)

## Wiring used by this package

### DWIN
- Mega `TX2 D16` -> TTL RX of `MAX3232`
- Mega `RX2 D17` <- TTL TX of `MAX3232`
- `MAX3232` RS232 side -> DWIN `R2/T2`
- DWIN power -> `12V`

### SC16IS752 #1
- `VCC` -> `3.3V`
- `GND` -> `GND`
- `RESET` -> `3.3V`
- `I2C/SPI` -> `GND`
- `SDA/VSS` -> `GND`
- `A0/CS` -> `D10`
- `A1/SI` -> `D51`
- `NC/SO` -> `D50`
- `SCL/SCLK` -> `D52`

### Local TTL↔RS422 in the cabinet
- power -> `3.3V`
- `TXA` from SC16 -> `RXD`
- `RXA` to SC16 <- `TXD`
- `GND` common

### RS422 pair between modules
- local `T+` -> remote `R+`
- local `T-` -> remote `R-`
- remote `T+` -> local `R+`
- remote `T-` -> local `R-`

### Remote TTL↔RS422 and laser
- remote module power -> `3.3V`
- laser power -> `3.3V`
- remote `TXD` -> laser `RX`
- remote `RXD` <- laser `TX`
- `GND` common

## DWIN VP map used now
- `0x1000` = X1
- `0x1002` = X2
- `0x1004` = Z1
- `0x1006` = Z2
- `0x1010` = MODE
- `0x1012` = ERROR
- `0x1100` = CMD

## Expected result

- Serial monitor shows stable `X1 = ... mm`
- DWIN field `X1` updates with the same value
- `ERROR` on DWIN stays `0` while the laser reads successfully

## Important note

This is a reliable baseline package.
It intentionally uses `single-shot` reads because that mode is already proven on your hardware.
After this step is stable, the next step will switch the laser driver to a faster non-blocking mode.

