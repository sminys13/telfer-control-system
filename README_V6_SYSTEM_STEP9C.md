# V6 SYSTEM STEP9C — DWIN PROGRAM / ZONE EDITOR

Base: Step9B (AUTO timing fix confirmed on real Mega: SIM-DEMO 21/21 DONE).

## What changed

Step9C does NOT change AutoRunnerV6 motion sequence. It adds a DWIN-side control layer for choosing program slots and zones and exposes editor status VPs.

Existing main DWIN button codes remain unchanged:
- 0x0001 MANUAL
- 0x0002 AUTO
- 0x0003 HOME
- 0x0004 STOP
- 0x0005 SETTINGS
- 0x0006 CALIBRATION

All buttons still write to VP_CMD = 0x1100.

## New touch commands (Return Key Code -> VP 0x1100)

| Function | Key value |
|---|---:|
| Program 1 | 0x0314 |
| Program 2 | 0x0315 |
| Program 3 | 0x0316 |
| Program 4 | 0x0317 |
| Previous zone | 0x0318 |
| Next zone | 0x0319 |
| Zone ON/OFF | 0x031A |

Existing editor commands:
- 0x0310 LOAD current program slot
- 0x0311 SAVE current program slot
- 0x0312 safe DEFAULTS in RAM
- 0x0320 CAPTURE HOME X1/X2
- 0x0321 CAPTURE TRAVEL Z1/Z2
- 0x0322 CAPTURE selected zone X1/X2
- 0x0323 CAPTURE selected zone Z1/Z2
- 0x0324 CAPTURE dryer X1/X2
- 0x0325 CAPTURE dryer Z1/Z2
- 0x0326 ACCEPT manually entered selected-zone X pair
- 0x0327 ACCEPT manually entered selected-zone Z pair
- 0x0328 ACCEPT manually entered HOME pair
- 0x0329 ACCEPT manually entered TRAVEL pair
- 0x032A ACCEPT manually entered dryer X pair
- 0x032B ACCEPT manually entered dryer Z pair

## Program editor VP map

Number-input DWIN objects should use Data auto-uploading so Mega receives writes.

| Field | VP |
|---|---:|
| Selected zone (1..10) | 0x1300 |
| Zone count | 0x1302 |
| Zone enabled | 0x1304 |
| Zone X1 | 0x1306 |
| Zone X2 | 0x1308 |
| Zone Z1 | 0x130A |
| Zone Z2 | 0x130C |
| Dip time, sec | 0x130E |
| Tilt step, mm | 0x1310 |
| Wait after tilt, sec | 0x1312 |
| Horizontal speed, % | 0x1314 |
| Vertical speed, % | 0x1316 |
| HOME X1 | 0x1320 |
| HOME X2 | 0x1322 |
| TRAVEL Z1 | 0x1324 |
| TRAVEL Z2 | 0x1326 |
| Drain wait, sec | 0x1328 |
| Low side (0=V1,1=V2) | 0x132A |
| Dryer enable | 0x1330 |
| Dryer time, sec | 0x1332 |
| Staging zone | 0x1334 |
| Dryer X1/X2 | 0x1336 / 0x1338 |
| Dryer Z1/Z2 | 0x133A / 0x133C |
| Program valid mask | 0x1340 |
| Selected-zone valid mask | 0x1342 |
| Program dirty | 0x1344 |
| Tilt speed, % | 0x1346 |
| Selected program slot (1..4) | 0x1348 |
| Program UI state | 0x134A |
| Editor locked | 0x134C |

### VP_PROG_UI_STATE 0x134A
- 0 idle
- 1 loaded
- 2 saved
- 3 changed / unsaved
- 4 error / rejected

### VP_PROG_EDIT_LOCKED 0x134C
- 0 editor may be used
- 1 AUTO/HOME is running; program edits are rejected

## Important validity rule

Typing only one coordinate of a pair does not make that pair calibrated.
After manual entry use the matching ACCEPT button (0x0326..0x032B), or use CAPTURE to read both sensors simultaneously.

This prevents a real AUTO run from treating an unknown X2/Z2 as a valid zero coordinate.

## SIM-DEMO safety

SIM-DEMO remains RAM-only even if fields are edited after `sim demo`.
It cannot be saved to program EEPROM.

## Suggested DWIN Program page controls

Without changing the background image, add DGUS objects over the prepared Program/Zone screen:
- four Program buttons: P1/P2/P3/P4 -> 0x0314..0x0317
- zone previous/next -> 0x0318 / 0x0319
- zone ON/OFF -> 0x031A
- Load / Save -> 0x0310 / 0x0311
- number inputs bound to 0x1300..0x1346
- Capture X and Capture Z -> 0x0322 / 0x0323
- Accept X and Accept Z -> 0x0326 / 0x0327

HOME/TRAVEL can be placed on a calibration/setup program subpage using 0x1320..0x1326 plus 0x0320/21 and 0x0328/29.

## Validation performed in this workspace

Host syntax check with `-Wall -Wextra -Werror`: all 14 active Step9C C++ sources OK.
AutoRunner regression:
- AUTO_SIM_OK totalSteps=21
- HOME_SIM_OK totalSteps=2
- DISABLED_ZONE_STEP_OK totalSteps=12
- DRYER_SIM_OK totalSteps=34

Actual AVR/PlatformIO compile/upload should still be performed on the user's VS Code/PlatformIO machine.
