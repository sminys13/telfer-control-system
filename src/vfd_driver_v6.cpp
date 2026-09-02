#include "vfd_driver_v6.h"
#include <string.h>

void VfdDriverV6::begin(const SettingsV6& settings) {
  _comm = settings.vfd;
  memcpy(_drive, settings.drive, sizeof(_drive));

#if defined(UBRR1H)
  _modbus.begin(Serial1,
                 PIN_VFD_RS485_DE_RE,
                 baud(),
                 _comm.responseTimeoutMs,
                 serialMode(),
                 _comm.retries,
                 VFD_RS485_ENABLED,
                 VFD_DRY_RUN || !VFD_RS485_ENABLED,
                 VFD_MODBUS_TRACE);
#else
  #error "V6 VFD layer requires Mega Serial1"
#endif

  _drives.begin(_modbus, settings);
  _begun = true;
  _status = (VFD_DRY_RUN || !VFD_RS485_ENABLED)
                ? VFD_STATUS_DRY_RUN
                : VFD_STATUS_READY;

  Serial.println(F("VfdDriverV6: existing Modbus/Drives layer connected"));
  if (!VFD_RS485_ENABLED) {
    Serial.println(F("VfdDriverV6: physical MAX485 disabled; exact NE200 frames will be printed"));
  } else if (!VFD_WRITE_COMMANDS_ENABLED) {
    if (HE200_COMMISSIONING)
      Serial.println(F("VfdDriverV6: HE200 physical MAX485 READ-ONLY; monitoring reads only, ALL writes blocked"));
    else
      Serial.println(F("VfdDriverV6: physical MAX485 enabled READ-ONLY; all register writes are blocked"));
  } else {
    Serial.println(F("VfdDriverV6: physical MAX485 enabled with WRITE commands"));
  }
  printConfiguration();
}

void VfdDriverV6::applySettings(const SettingsV6& settings) {
  _comm = settings.vfd;
  memcpy(_drive, settings.drive, sizeof(_drive));

  if (_begun) {
    _modbus.reconfigure(baud(),
                        _comm.responseTimeoutMs,
                        serialMode(),
                        _comm.retries,
                        VFD_RS485_ENABLED,
                        VFD_DRY_RUN || !VFD_RS485_ENABLED,
                        VFD_MODBUS_TRACE);
    _drives.applySettings(settings);
  }

  Serial.println(F("VFD runtime settings applied:"));
  printConfiguration();
}

bool VfdDriverV6::service(uint32_t nowMs) {
  return _drives.tick(nowMs);
}

uint16_t VfdDriverV6::serialMode() const {
  const bool twoStop = (_comm.stopBits == 2);
  switch (_comm.parity) {
    case VFD_PARITY_EVEN: return twoStop ? SERIAL_8E2 : SERIAL_8E1;
    case VFD_PARITY_ODD:  return twoStop ? SERIAL_8O2 : SERIAL_8O1;
    default:              return twoStop ? SERIAL_8N2 : SERIAL_8N1;
  }
}

void VfdDriverV6::printConfiguration() const {
  Serial.print(F("  baud="));
  Serial.print(baud());
  Serial.print(F(" parity="));
  Serial.print(_comm.parity);
  Serial.print(F(" stopBits="));
  Serial.print(_comm.stopBits);
  Serial.print(F(" timeout="));
  Serial.print(_comm.responseTimeoutMs);
  Serial.print(F("ms retries="));
  Serial.print(_comm.retries);
  Serial.print(F(" interRequest="));
  Serial.print(_comm.interRequestMs);
  Serial.println(F("ms"));

  Serial.print(F("  addresses H1/H2/V1/V2="));
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    if (i) Serial.print('/');
    Serial.print(_comm.address[i]);
    if (_comm.invertDirectionMask & (1U << i)) Serial.print(F("i"));
  }
  Serial.print(F(" watchdog="));
  Serial.print(_comm.manualJogTimeoutMs);
  Serial.println(F("ms"));
}

uint8_t VfdDriverV6::connectedCount() const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    if (_drives.telemetry((DriveId)i).connected) ++n;
  }
  return n;
}

uint8_t VfdDriverV6::address(uint8_t driveIndex) const {
  if (driveIndex >= DRIVE_COUNT_V6) return 0;
  return _comm.address[driveIndex];
}

int16_t VfdDriverV6::manualPercent(uint8_t driveIndex) const {
  if (driveIndex >= DRIVE_COUNT_V6) return 0;
  return (int16_t)_drive[driveIndex].manualPercent;
}

void VfdDriverV6::setExclusiveTargets(int16_t h1, int16_t h2,
                                      int16_t v1, int16_t v2) {
  _drives.setSpeed(DriveId::H1, h1);
  _drives.setSpeed(DriveId::H2, h2);
  _drives.setSpeed(DriveId::V1, v1);
  _drives.setSpeed(DriveId::V2, v2);
}

void VfdDriverV6::printMovePlan(uint16_t motorStateCode) const {
  Serial.print(F("NE200 MOVE PLAN state="));
  Serial.print(motorStateCode);
  Serial.print(F(" manual% H1/H2/V1/V2="));
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    if (i) Serial.print('/');
    Serial.print(_drive[i].manualPercent);
  }
  Serial.println();
}

void VfdDriverV6::startByMotorState(uint16_t motorStateCode) {
  if (VFD_RS485_ENABLED && !VFD_WRITE_COMMANDS_ENABLED) {
    _status = VFD_STATUS_BLOCKED;
    Serial.println(F("NE200 WRITE BLOCKED by read-only build: movement command not transmitted"));
    return;
  }

  const int16_t h1 = manualPercent(DRIVE_H1);
  const int16_t h2 = manualPercent(DRIVE_H2);
  const int16_t v1 = manualPercent(DRIVE_V1);
  const int16_t v2 = manualPercent(DRIVE_V2);

  printMovePlan(motorStateCode);

  // Existing project convention:
  //   horizontal positive = forward/right;
  //   vertical drive Forward = down, therefore UP uses a negative request.
  switch (motorStateCode) {
    case MOTOR_STATE_H1_FWD:      setExclusiveTargets(+h1, 0, 0, 0); break;
    case MOTOR_STATE_H1_BWD:      setExclusiveTargets(-h1, 0, 0, 0); break;
    case MOTOR_STATE_H2_FWD:      setExclusiveTargets(0, +h2, 0, 0); break;
    case MOTOR_STATE_H2_BWD:      setExclusiveTargets(0, -h2, 0, 0); break;
    case MOTOR_STATE_H_BOTH_FWD:  setExclusiveTargets(+h1, +h2, 0, 0); break;
    case MOTOR_STATE_H_BOTH_BWD:  setExclusiveTargets(-h1, -h2, 0, 0); break;
    case MOTOR_STATE_V1_UP:       setExclusiveTargets(0, 0, -v1, 0); break;
    case MOTOR_STATE_V1_DOWN:     setExclusiveTargets(0, 0, +v1, 0); break;
    case MOTOR_STATE_V2_UP:       setExclusiveTargets(0, 0, 0, -v2); break;
    case MOTOR_STATE_V2_DOWN:     setExclusiveTargets(0, 0, 0, +v2); break;
    case MOTOR_STATE_V_BOTH_UP:   setExclusiveTargets(0, 0, -v1, -v2); break;
    case MOTOR_STATE_V_BOTH_DOWN: setExclusiveTargets(0, 0, +v1, +v2); break;
    default:
      _drives.stopAll();
      _status = VFD_STATUS_BLOCKED;
      Serial.println(F("NE200 MOVE PLAN rejected: unknown motor state"));
      return;
  }

  _status = (VFD_DRY_RUN || !VFD_RS485_ENABLED)
                ? VFD_STATUS_DRY_RUN
                : VFD_STATUS_MOVING;
  Serial.println(F("NE200 command queued; scheduler will emit one RTU transaction per step"));
}

bool VfdDriverV6::setAutoLogicalTargets(int16_t h1RightPct, int16_t h2RightPct,
                                             int16_t v1UpPct, int16_t v2UpPct) {
  if (VFD_RS485_ENABLED && !VFD_WRITE_COMMANDS_ENABLED) {
    _status = VFD_STATUS_BLOCKED;
    if (h1RightPct || h2RightPct || v1UpPct || v2UpPct)
      Serial.println(F("AUTO WRITE BLOCKED by physical RS485 READ-ONLY build"));
    return false;
  }

  int16_t logical[DRIVE_COUNT_V6] = {h1RightPct, h2RightPct, v1UpPct, v2UpPct};
  bool changed = !_autoTargetKnown;
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    if (logical[i] > 100) logical[i] = 100;
    if (logical[i] < -100) logical[i] = -100;
    if (_lastAutoLogical[i] != logical[i]) changed = true;
  }

  if (changed) {
    Serial.print(F("AUTO TARGET logical H1/H2/V1up/V2up="));
    for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
      if (i) Serial.print('/');
      Serial.print(logical[i]);
      _lastAutoLogical[i] = logical[i];
    }
    Serial.println();
    _autoTargetKnown = true;
  }

  // Existing NE200 convention: positive physical V request is Forward=DOWN.
  // Automatic logic works in a much safer coordinate convention where +V means UP,
  // hence the sign inversion for V1/V2 here.
  _drives.setSpeed(DriveId::H1, logical[DRIVE_H1]);
  _drives.setSpeed(DriveId::H2, logical[DRIVE_H2]);
  _drives.setSpeed(DriveId::V1, (int16_t)-logical[DRIVE_V1]);
  _drives.setSpeed(DriveId::V2, (int16_t)-logical[DRIVE_V2]);

  const bool moving = logical[0] || logical[1] || logical[2] || logical[3];
  _status = (VFD_DRY_RUN || !VFD_RS485_ENABLED)
                ? VFD_STATUS_DRY_RUN
                : (moving ? VFD_STATUS_MOVING : VFD_STATUS_READY);
  return true;
}

void VfdDriverV6::stopAll(const __FlashStringHelper* reason) {
  _autoTargetKnown = false;
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) _lastAutoLogical[i] = 0;
  if (!VFD_RS485_ENABLED || VFD_WRITE_COMMANDS_ENABLED) {
    _drives.stopAll();
  }
  _status = VFD_STATUS_STOPPED;

  Serial.print(F("NE200 STOP ALL queued"));
  if (reason) {
    Serial.print(F(" reason="));
    Serial.print(reason);
  }
  if (!VFD_RS485_ENABLED || VFD_WRITE_COMMANDS_ENABLED)
    Serial.println(F("; each drive gets STOP then zero setpoint"));
  else
    Serial.println(F("; READ-ONLY build, no STOP/SETPOINT frame transmitted"));
}

void VfdDriverV6::block(const __FlashStringHelper* reason) {
  if (!VFD_RS485_ENABLED || VFD_WRITE_COMMANDS_ENABLED) _drives.stopAll();
  _status = VFD_STATUS_BLOCKED;
  Serial.print(F("VFD BLOCKED"));
  if (reason) {
    Serial.print(F(" reason="));
    Serial.print(reason);
  }
  Serial.println();
}

void VfdDriverV6::testConnection(uint8_t driveIndex) {
  _status = (VFD_DRY_RUN || !VFD_RS485_ENABLED)
                ? VFD_STATUS_DRY_RUN
                : VFD_STATUS_READY;

  if (driveIndex >= DRIVE_COUNT_V6) {
    Serial.println(HE200_COMMISSIONING
                       ? F("HE200 READ-ONLY TEST ALL queued: FC03 monitoring 0x7000/0x702D/0x703B")
                       : F("LEGACY SAFE TEST ALL queued"));
    _drives.requestSafeStatusReadAll();
    return;
  }

  Serial.print(HE200_COMMISSIONING ? F("HE200 READ-ONLY TEST queued drive=") : F("LEGACY SAFE TEST queued drive="));
  Serial.print(Drives::driveName((DriveId)driveIndex));
  Serial.print(F(" addr="));
  Serial.print(address(driveIndex));
  Serial.println(HE200_COMMISSIONING
                     ? F(" FC03 0x7000..0x7007 + 0x702D + 0x703B..0x703D")
                     : F(" legacy map"));
  _drives.requestSafeStatusRead((DriveId)driveIndex);
}
