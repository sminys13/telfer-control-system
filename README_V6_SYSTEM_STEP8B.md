# V6 System Step8B — configurable bench safety + physical RS485 READ-ONLY commissioning

## What changed from Step8A

1. E-stop and horizontal limit monitoring can be enabled/disabled at runtime in BENCH mode.
2. Existing EEPROM v2 layout is preserved. The former reserved byte is now `safetyDisableMask`:
   - bit0 = disable E-stop monitoring
   - bit1 = disable horizontal limit monitoring
   - 0 = both enabled (old EEPROM therefore stays safe/compatible)
3. DWIN/service commands:
   - `0x0047` clear E-stop latch (only when physical E-stop input is inactive)
   - `0x0048` toggle E-stop monitoring (BENCH only)
   - `0x0049` toggle horizontal limit monitoring (BENCH only)
   - use `SAVE` (`0x0030` or settings save `0x0201`) to persist a toggle
4. New editable VP values:
   - `0x1222` E-stop monitor 0/1
   - `0x1224` limit monitor 0/1
5. New header/status VPs:
   - `0x1052` effective E-stop monitor state 0/1
   - `0x1054` effective limit monitor state 0/1
6. FIELD/NC build refuses software attempts to disable either safety function.
7. Added a second PlatformIO environment `mega_v6_ne200_readonly`.

## Mega2560 ↔ MAX485 wiring

- Mega TX1 pin 18 -> MAX485 DI
- Mega RX1 pin 19 <- MAX485 RO
- Mega pin 6 -> MAX485 DE + /RE tied together
- Mega GND <-> MAX485 GND
- MAX485 A/B -> NE200 RS485 A/B bus
- Pin 7 is only a legacy reserve and is not used by Step8B.

## Default environment — still no physical RS485

Build/upload:

`mega_v6_sensor_core_fast`

This remains dry-run and safe. Exact RTU frames are printed only.

## How to get out of STOP after testing E-stop

After E-stop is RELEASED:

1. Send DWIN command `0x0047 SAFETY CLEAR`, or use the existing `0x0044 CLEAR STATUS` button/command.
2. The terminal should print `SAFETY E-stop latch cleared`.
3. Then select MANUAL/AUTO/HOME.

If you do not have a physical E-stop during bench work, send `0x0048` once to disable E-stop monitoring in RAM. Send `SAVE` only if you intentionally want that bench setting to survive reboot.

If you do not have horizontal limit switches, send `0x0049` once to disable them in RAM. Again, SAVE is optional.

## Physical RS485 read-only test

After soldering MAX485, build/upload:

`mega_v6_ne200_readonly`

Expected boot line:

`VfdDriverV6: physical MAX485 enabled READ-ONLY; all register writes are blocked`

In this environment:

- manual jog commands are blocked before any NE200 write;
- STOP does not write STOP/setpoint registers;
- only safe status read requests from `0x0204..0x0208` are intended.

For one connected NE200 at address 1 use `0x0204 TEST H1` first. The request is FC03 status read. If the drive answers, the terminal should show `NE200 READ OK ...` and VFD connected count should become 1.

Do not use the future write-enabled environment until A/B polarity, address, baud/parity and status reads have been verified.

## Current laser test setup

One physical laser is sufficient. X1/X2/Z1/Z2 are already independent channels through two SC16IS752 devices. Move the same laser connector/channel between inputs as needed; missing channels will remain `a=9999` and are expected during bench work.

## USB / PlatformIO service console

Set the terminal input to send a newline (`send_on_enter` is fine), then type:

- `help`
- `clear` — clear released E-stop latch
- `diag`
- `estop off` / `estop on` — BENCH only
- `limits off` / `limits on` — BENCH only
- `save` — persist current settings to EEPROM
- `test h1`, `test h2`, `test v1`, `test v2`, `test all`

This means no new physical button and no new DWIN button is required for bench commissioning.
