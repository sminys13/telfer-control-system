# V6 System Step9A — AUTO/HOME + programs + safe simulation

Step9A continues directly from Step8B. The sensor/DWIN/NE200 layers are kept; this step restores the old technological AUTO sequence and adapts it to the new four-laser coordinate model:

- horizontal coordinates: `X1`, `X2`;
- vertical coordinates: `Z1`, `Z2` (replaces legacy `US1`, `US2` targets);
- H positive = right;
- logical V positive = up; the NE200 Forward=DOWN sign conversion stays inside `VfdDriverV6`.

## What is implemented

### AUTO process sequence

The legacy sequence from `src/app.cpp` is preserved:

1. move both vertical sides to transport Z;
2. move H1/H2 to the current zone X;
3. lower the configured LOW side by the tilt step;
4. wait after tilt;
5. lower both sides to the zone Z targets;
6. immersion dwell;
7. raise the opposite HIGH side by the tilt step;
8. drain wait;
9. raise both sides to transport Z;
10. continue with the next enabled zone;
11. optional dryer workflow with operator confirmations;
12. HOME: transport Z first, then X1/X2 home.

Every movement phase has a 180 s timeout. A lost required sensor, E-stop or a blocking horizontal limit faults AUTO and stops the targets.

The old separate tilt speed is retained: `tiltPercent`, default **35%**.

### Programs

- 4 program slots, same as the legacy project;
- up to 10 zones, same as the legacy project;
- per-zone X1/X2 and Z1/Z2 targets;
- dip time, tilt step, post-tilt wait, horizontal and vertical speed caps;
- enabled/disabled zone and ordered zone array;
- HOME X1/X2;
- transport Z1/Z2;
- drain time;
- LOW side selection V1/V2;
- optional dryer X/Z, timer and staging zone;
- CRC and explicit calibration-valid flags.

Program data is stored separately from `SettingsV6` (meta at EEPROM 384, slots beginning at 512). An empty/corrupt slot loads **uncalibrated safe defaults in RAM** and cannot start physical AUTO.

A numeric edit of only X1 or only X2 (or one Z) does **not** make an uncalibrated pair valid. Both values must be set as a pair, captured as a pair, or explicitly accepted from DWIN.

### DWIN runtime

The permanent header now receives real AUTO state:

- program slot;
- current/total zone;
- current/total step;
- elapsed time;
- known timed dwell total;
- AUTO phase/error/running/paused/simulation/operator wait;
- remaining wait seconds;
- program Ready flag.

AUTO runtime VPs: `0x1060..0x1070`.
Program editor VPs: `0x1300..0x1346`.
Program/AUTO commands: `0x0300..0x032B`.

### Controller limit switches

E-stop remains mandatory in FIELD builds and cannot be software-disabled there.

Controller-side horizontal limit monitoring **can now be enabled or disabled even in FIELD**, because the final cabinet may implement end limits as a hardwired protection outside the Mega. Use `limits off` / `limits on`, then `save` if the choice must survive reboot. The effective state is always sent to DWIN.

## Build environments / commissioning ladder

### 1. `mega_v6_sensor_core_fast` — today's safe bench build

- MAX485 physical transport disabled;
- manual commands only print dry-run RTU frames;
- full AUTO/HOME simulation allowed;
- bench E-stop/limit monitoring may be disabled from the console.

### 2. `mega_v6_ne200_readonly` — first real RS485 test

- MAX485 / Serial1 physically enabled;
- register writes and motion are blocked;
- use `test h1`, `test h2`, `test v1`, `test v2`, `test all`.

### 3. `mega_v6_ne200_field_manual` — real manual commissioning

- FIELD/NC safety interpretation;
- real NE200 writes enabled;
- MANUAL motion allowed;
- physical HOME/AUTO is compile-time blocked.

Use this after READ-ONLY communication is confirmed. Verify each direction separately before proceeding.

### 4. `mega_v6_ne200_field_auto` — full physical AUTO/HOME

- FIELD/NC safety interpretation;
- NE200 writes enabled;
- explicit physical AUTO/HOME compile-time enable.

Use only after RS485, drive directions, E-stop, required limits (if controller-side), four sensor channels and program coordinates have all been checked.

The firmware contains `static_assert` interlocks so a physical-write build cannot be compiled with BENCH safety logic.

## Today's recommended test

Build/upload **`mega_v6_sensor_core_fast`** and open PlatformIO Monitor at 115200.

If no physical safety inputs are connected on the bench:

```text
clear
estop off
limits off
sim demo
auto
```

`sim demo` loads a **RAM-only**, two-zone calibrated example and enables simulation. It deliberately cannot be saved to EEPROM.

Expected result:

- `AUTO START ... SIMULATION`;
- phase transitions through the complete process;
- DWIN X1/X2/Z1/Z2 show simulated movement while AUTO is running;
- demo has 21 steps;
- terminal message `AUTO PROGRAM DONE`;
- system returns to STOP without transmitting physical RS485 data.

Useful during the run:

```text
pause
resume
stop
diag
```

`home` starts a HOME-only sequence (2 steps).

## USB console — Step9A additions

```text
sim on
sim off
sim demo
auto
home
stop
pause
resume
next

prog show
prog slot 1
prog load
prog save
prog defaults

set home 1000 1000
set travel 1000 1000
set zone 1 x 2000 2000
set zone 1 z 500 500
zone 1 dip 60
zone 1 wait 30
zone 1 tilt 30
zone 1 speed 55 45
tilt speed 35
zone 1 on

cap home
cap travel
cap zone 1 x
cap zone 1 z
cap dry x
cap dry z
```

With only one physical laser, pair capture (`cap ...`) is expected to reject because both channels of the pair must be simultaneously valid. For bench preparation you can use the numeric `set ...` commands. **Physical AUTO still requires all four X1/X2/Z1/Z2 channels usable at the same time.**

## DWIN capture / accept commands

```text
0x0320 CAP HOME X1/X2
0x0321 CAP TRAVEL Z1/Z2
0x0322 CAP current zone X1/X2
0x0323 CAP current zone Z1/Z2
0x0324 CAP DRY X1/X2
0x0325 CAP DRY Z1/Z2
0x0326 ACCEPT numeric current-zone X pair
0x0327 ACCEPT numeric current-zone Z pair
0x0328 ACCEPT numeric HOME pair
0x0329 ACCEPT numeric TRAVEL pair
0x032A ACCEPT numeric DRY X pair
0x032B ACCEPT numeric DRY Z pair
```

## MAX485 reminder

Mega 2560 Serial1:

```text
pin 18 TX1 -> MAX485 DI
pin 19 RX1 <- MAX485 RO
pin 6      -> MAX485 DE + /RE tied together
GND        -> common GND
A/B        -> NE200 RS485 bus
```

Current defaults: 9600 baud, even parity, 1 stop bit; drive addresses H1/H2/V1/V2 = 1/2/3/4.

## FIELD/NC safety wiring used by the firmware

In FIELD builds the input uses `INPUT_PULLUP` and NC-to-GND logic:

- healthy closed NC contact -> LOW;
- open/pressed/broken wire -> HIGH -> active fault.

E-stop: Mega pin 2.
Controller-side limits if enabled: H1 L/R = 36/37, H2 L/R = 38/39.

If limits are implemented only as external/hardwired protection, disable only the **controller limit monitor** with `limits off`. Do not bypass the field E-stop chain.

## Verification performed before packaging

The changed active sources were host-syntax-checked with warnings enabled. The AutoRunner was also executed with an Arduino host stub:

- two-zone demo: `AUTO_SIM_OK`, 21 steps;
- HOME-only: 2 steps;
- disabled-zone test: monotonic steps, 12 total;
- dryer sequence with all operator gates: `DRYER_SIM_OK`, 34 steps.

A real AVR/PlatformIO build still must be confirmed on the user's workstation because PlatformIO/avr-g++ is not installed in the packaging environment.
