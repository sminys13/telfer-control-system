#include "vfd_driver_v6.h"
#include <string.h>
#include "web_control_policy_v6.h"

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
                 VFD_MODBUS_TRACE,
                 RS485_AUTO_DIRECTION);
#else
  #error "V6 VFD layer requires Mega Serial1"
#endif

  _drives.begin(_modbus, settings);
  _begun = true;
  _status = (VFD_DRY_RUN || !VFD_RS485_ENABLED)
                ? VFD_STATUS_DRY_RUN
                : VFD_STATUS_READY;

  Serial.println(F("VfdDriverV6: existing Modbus/Drives layer connected"));
  Serial.print(F("VfdDriverV6: RS485 direction="));
  Serial.println(RS485_AUTO_DIRECTION ? F("AUTO (isolated Waveshare, pin 6 unused)")
                                       : F("MANUAL DE/RE (MAX485 pin 6)"));
  if (HE200_DIAGNOSTIC_LOCK && VFD_RS485_ENABLED) {
    Serial.println(F("HE200 DIAGNOSTIC LOCK: all physical FC06 writes blocked, including software STOP"));
  } else if (!VFD_RS485_ENABLED) {
    Serial.println(F("VfdDriverV6: physical MAX485 disabled; exact NE200 frames will be printed"));
  } else if (!VFD_WRITE_COMMANDS_ENABLED) {
    if (HE200_COMMISSIONING)
      Serial.println(F("VfdDriverV6: HE200 physical RS485 READ-ONLY; monitoring reads only, ALL writes blocked"));
    else
      Serial.println(F("VfdDriverV6: physical MAX485 enabled READ-ONLY; all register writes are blocked"));
  } else {
    Serial.println(WEB_CONTROL_ENABLED ? F("VfdDriverV6: HE200 WEB CONTROL; motion requires runtime arm and measured directions") : HE200_FIELD_SERVICE ? F("VfdDriverV6: HE200 FIELD SERVICE writes enabled via isolated Waveshare; AUTO/HOME remain blocked") : F("VfdDriverV6: physical RS485 WRITE commands enabled"));
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
                        VFD_MODBUS_TRACE,
                        RS485_AUTO_DIRECTION);
    _drives.applySettings(settings);
  }

  Serial.println(F("VFD runtime settings applied:"));
  printConfiguration();
}

bool VfdDriverV6::service(uint32_t nowMs) {
  if (_audit.active()) return _audit.tick(nowMs);
  return _drives.tick(nowMs);
}

bool VfdDriverV6::setServiceRawTargets(const int16_t targets[4]) {
  if (HE200_DIAGNOSTIC_LOCK || !HE200_FIELD_SERVICE || _audit.active()) return false;
  if (WEB_CONTROL_ENABLED && !_webPermit) return false;
  uint8_t moving=0;
  for (uint8_t i=0;i<4;++i) { if (targets[i]) ++moving; if (targets[i]>20 || targets[i]<-20) return false; }
  if (moving>1 && !WEB_CONTROL_ENABLED) return false;
  for (uint8_t i=0;i<4;++i) {
    // Cancel the legacy EEPROM drive inversion; this interface names actual FWD/REV.
    _drives.setSpeed((DriveId)i,(_comm.invertDirectionMask & (1U<<i)) ? -targets[i] : targets[i]);
  }
  _autoTargetKnown=false;
  return true;
}

bool VfdDriverV6::startHe200Audit() {
  if (!_begun || !HE200_COMMISSIONING || _drives.hasPendingWork()) return false;
  for (uint8_t i=0;i<4;++i) if (_drives.targetPct((DriveId)i)) return false;
  uint8_t addresses[4];
  for (uint8_t i=0;i<4;++i) addresses[i]=address(i);
  return _audit.start(_modbus,addresses);
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
  if(WEB_CONTROL_ENABLED){
    const int16_t logical[4]={h1RightPct,h2RightPct,v1UpPct,v2UpPct};
    for(uint8_t i=0;i<4;i++)if(logical[i] && (!_webPermit || !_coordinateSigns[i]))return false;
    for(uint8_t i=0;i<4;i++) {
      int16_t raw=webCoordinatePercent(logical[i],_coordinateSigns[i],(_comm.invertDirectionMask&(1u<<i))!=0,_drive[i].maxPercent);
      _drives.setSpeed((DriveId)i,raw);
    }
    _status=(logical[0]||logical[1]||logical[2]||logical[3])?VFD_STATUS_MOVING:VFD_STATUS_READY;
    return true;
  }
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

ModbusResult VfdDriverV6::readParameter(uint8_t drive,uint16_t reg,uint16_t& value) {
  if(drive>=4 || !_begun || _audit.active() || _drives.hasPendingWork())return {false,MODBUS_ERROR_BAD_RESPONSE,false,0};
  return _modbus.readHoldingRegisters(address(drive),reg,1,&value);
}
ModbusResult VfdDriverV6::writeParameter(uint8_t drive,uint16_t reg,uint16_t value) {
  if(!WEB_CONTROL_ENABLED || !_webPermit || drive>=4 || !_begun || _audit.active() || _drives.hasPendingWork())return {false,MODBUS_ERROR_WRITE_LOCKED,false,0};
  return _modbus.writeSingleRegister(address(drive),reg,value);
}

void VfdDriverV6::stopAll(const __FlashStringHelper* reason) {
  _autoTargetKnown = false;
  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) _lastAutoLogical[i] = 0;
  if (!VFD_RS485_ENABLED || VFD_WRITE_COMMANDS_ENABLED) {
    _drives.stopAll();
  }
  _status = VFD_STATUS_STOPPED;

  Serial.print(HE200_NATIVE_PROTOCOL ? F("HE200 DECEL STOP ALL queued") : F("NE200 STOP ALL queued"));
  if (reason) {
    Serial.print(F(" reason="));
    Serial.print(reason);
  }
  if (!VFD_RS485_ENABLED || VFD_WRITE_COMMANDS_ENABLED)
    Serial.println(HE200_NATIVE_PROTOCOL ? F("; each drive gets 0x2000=0006 deceleration stop") : F("; each drive gets STOP then zero setpoint"));
  else
    Serial.println(F("; READ-ONLY build, no STOP/SETPOINT frame transmitted"));
}

void VfdDriverV6::stopMask(uint8_t driveMask, const __FlashStringHelper* reason) {
  const uint8_t validMask = (uint8_t)((1u << DRIVE_COUNT_V6) - 1u);
  driveMask &= validMask;
  if (!driveMask) return;

  for (uint8_t i = 0; i < DRIVE_COUNT_V6; ++i) {
    if (!(driveMask & (uint8_t)(1u << i))) continue;
    _lastAutoLogical[i] = 0;
    if (!VFD_RS485_ENABLED || VFD_WRITE_COMMANDS_ENABLED)
      _drives.stop((DriveId)i);
  }

  if (driveMask == validMask) _autoTargetKnown = false;
  _status = (driveMask == validMask) ? VFD_STATUS_STOPPED : VFD_STATUS_READY;

  Serial.print(HE200_NATIVE_PROTOCOL ? F("HE200 DECEL STOP MASK queued mask=0x")
                                      : F("NE200 STOP MASK queued mask=0x"));
  Serial.print(driveMask, HEX);
  if (reason) {
    Serial.print(F(" reason="));
    Serial.print(reason);
  }
  if (!VFD_RS485_ENABLED || VFD_WRITE_COMMANDS_ENABLED)
    Serial.println(HE200_NATIVE_PROTOCOL ? F("; selected drives get 0x2000=0006")
                                         : F("; selected drives get STOP"));
  else
    Serial.println(F("; READ-ONLY build, no frame transmitted"));
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


const DriveTelemetry& VfdDriverV6::telemetry(uint8_t driveIndex) const {
  static DriveTelemetry empty{};
  if (driveIndex >= DRIVE_COUNT_V6) return empty;
  return _drives.telemetry((DriveId)driveIndex);
}

bool VfdDriverV6::readHe200ProtocolSnapshot(uint8_t driveIndex, He200ProtocolSnapshotV6& out) {
  if (driveIndex >= DRIVE_COUNT_V6 || !_begun || !HE200_NATIVE_PROTOCOL || !VFD_RS485_ENABLED) {
    Serial.println(F("HE200 protocol snapshot rejected: build/drive not eligible"));
    return false;
  }
  if (_drives.hasPendingWork()) {
    Serial.println(F("HE200 protocol snapshot BUSY: wait for scheduler"));
    return false;
  }

  const uint8_t addr = address(driveIndex);
  uint16_t v = 0;
  uint16_t state[3] = {0,0,0};
  ModbusResult r;

  r = _modbus.readHoldingRegisters(addr, HE200_REG_COMM_SETPOINT, 1, &v);
  if (!r.ok) return false;
  out.commSetpoint001Pct = v;

  r = _modbus.readHoldingRegisters(addr, HE200_REG_STATUS_WORD, 1, &v);
  if (!r.ok) return false;
  out.status3000 = v;

  r = _modbus.readHoldingRegisters(addr, HE200_REG_COMM_VALUE, 1, &v);
  if (!r.ok) return false;
  out.commValue701C = v;

  r = _modbus.readHoldingRegisters(addr, HE200_REG_CUR_SET_FREQ, 3, state);
  if (!r.ok) return false;
  out.setPct703B = state[0];
  out.runPct703C = (int16_t)state[1];
  out.runState703D = state[2];

  r = _modbus.readHoldingRegisters(addr, HE200_REG_RUNNING_FREQ, 1, &v);
  if (!r.ok) return false;
  out.runningFreq001Hz = v;

  r = _modbus.readHoldingRegisters(addr, HE200_REG_FAULT_INFO, 1, &v);
  if (!r.ok) return false;
  out.fault702D = v;

  Serial.print(F("@HE200_PROTOCOL drive=")); Serial.print(Drives::driveName((DriveId)driveIndex));
  Serial.print(F(" addr=")); Serial.print(addr);
  Serial.print(F(" set1000=")); Serial.print(out.commSetpoint001Pct);
  Serial.print(F(" status3000=")); Serial.print(out.status3000);
  Serial.print(F(" comm701C=")); Serial.print(out.commValue701C);
  Serial.print(F(" set703B=")); Serial.print(out.setPct703B);
  Serial.print(F(" run703C=")); Serial.print(out.runPct703C);
  Serial.print(F(" state703D=")); Serial.print(out.runState703D);
  Serial.print(F(" runHz001=")); Serial.print(out.runningFreq001Hz);
  Serial.print(F(" fault702D=")); Serial.println(out.fault702D);
  return true;
}

bool VfdDriverV6::writeHe200CommSetpoint(uint8_t driveIndex, uint16_t pct001) {
  if (driveIndex >= DRIVE_COUNT_V6 || !_begun || !HE200_NATIVE_PROTOCOL ||
      !VFD_RS485_ENABLED || !VFD_WRITE_COMMANDS_ENABLED || !HE200_FIELD_SERVICE) {
    Serial.println(F("HE200 WRITE BLOCKED: field service build required"));
    return false;
  }
  if (pct001 > HE200_SETPOINT_MAX) pct001 = HE200_SETPOINT_MAX;
  if (_drives.hasPendingWork()) {
    Serial.println(F("HE200 WRITE BUSY: scheduler has pending work"));
    return false;
  }
  const ModbusResult r = _modbus.writeSingleRegister(address(driveIndex), HE200_REG_COMM_SETPOINT, pct001);
  Serial.print(F("@HE200_WRITE drive=")); Serial.print(Drives::driveName((DriveId)driveIndex));
  Serial.print(F(" reg=0x1000 value=")); Serial.print(pct001);
  Serial.print(F(" ok=")); Serial.println(r.ok ? 1 : 0);
  return r.ok;
}

bool VfdDriverV6::writeHe200Control(uint8_t driveIndex, uint16_t command) {
  if (driveIndex >= DRIVE_COUNT_V6 || !_begun || !HE200_NATIVE_PROTOCOL ||
      !VFD_RS485_ENABLED || !VFD_WRITE_COMMANDS_ENABLED || !HE200_FIELD_SERVICE) {
    Serial.println(F("HE200 CONTROL BLOCKED: field service build required"));
    return false;
  }
  // Service layer deliberately exposes only the verified command subset.
  if (command != HE200_CMD_FORWARD && command != HE200_CMD_REVERSE &&
      command != HE200_CMD_DECEL_STOP && command != HE200_CMD_RESET_FAULT) {
    Serial.println(F("HE200 CONTROL rejected: command not whitelisted"));
    return false;
  }
  if (_drives.hasPendingWork()) {
    Serial.println(F("HE200 CONTROL BUSY: scheduler has pending work"));
    return false;
  }
  const ModbusResult r = _modbus.writeSingleRegister(address(driveIndex), HE200_REG_CONTROL_WORD, command);
  Serial.print(F("@HE200_WRITE drive=")); Serial.print(Drives::driveName((DriveId)driveIndex));
  Serial.print(F(" reg=0x2000 value=")); Serial.print(command);
  Serial.print(F(" ok=")); Serial.println(r.ok ? 1 : 0);
  return r.ok;
}

bool VfdDriverV6::probeHe200Protocol(uint8_t driveIndex, uint16_t probePct001,
                                     He200ProtocolSnapshotV6& outAfter) {
  if (!HE200_FIELD_SERVICE || !VFD_WRITE_COMMANDS_ENABLED) {
    Serial.println(F("@GATE name=PROTOCOL result=FAIL reason=FIELD_SERVICE_BUILD_REQUIRED"));
    return false;
  }
  He200ProtocolSnapshotV6 before{};
  if (!readHe200ProtocolSnapshot(driveIndex, before)) {
    Serial.println(F("@GATE name=PROTOCOL result=FAIL reason=READ_BEFORE"));
    return false;
  }
  // Never probe a running/faulted drive. 0x703D=0 is the observed STOP state.
  if (before.runningFreq001Hz > 20 || before.runState703D != 0 || before.fault702D != 0) {
    Serial.print(F("@GATE name=PROTOCOL drive=")); Serial.print(Drives::driveName((DriveId)driveIndex));
    Serial.println(F(" result=FAIL reason=DRIVE_NOT_STOPPED_OR_FAULT"));
    return false;
  }
  if (probePct001 > 1000) probePct001 = 1000; // max 10% during protocol proof
  if (probePct001 == 0) probePct001 = 500;    // default 5%

  if (!writeHe200CommSetpoint(driveIndex, probePct001)) {
    Serial.println(F("@GATE name=PROTOCOL result=FAIL reason=WRITE_1000"));
    return false;
  }

  uint16_t comm = 0;
  ModbusResult r = _modbus.readHoldingRegisters(address(driveIndex), HE200_REG_COMM_VALUE, 1, &comm);
  if (!r.ok || comm != probePct001) {
    (void)writeHe200CommSetpoint(driveIndex, before.commSetpoint001Pct);
    Serial.print(F("@GATE name=PROTOCOL drive=")); Serial.print(Drives::driveName((DriveId)driveIndex));
    Serial.print(F(" result=FAIL reason=VERIFY_701C expected=")); Serial.print(probePct001);
    Serial.print(F(" got=")); Serial.println(comm);
    return false;
  }

  // A deceleration-stop command on an already stopped drive is the safest
  // possible proof of the 0x2000 command register.
  if (!writeHe200Control(driveIndex, HE200_CMD_DECEL_STOP)) {
    (void)writeHe200CommSetpoint(driveIndex, before.commSetpoint001Pct);
    Serial.println(F("@GATE name=PROTOCOL result=FAIL reason=WRITE_2000"));
    return false;
  }
  (void)writeHe200CommSetpoint(driveIndex, before.commSetpoint001Pct);

  delay(30);
  if (!readHe200ProtocolSnapshot(driveIndex, outAfter)) {
    Serial.println(F("@GATE name=PROTOCOL result=FAIL reason=READ_AFTER"));
    return false;
  }
  const bool stopped = outAfter.runningFreq001Hz <= 20 && outAfter.runState703D == 0;
  Serial.print(F("@GATE name=PROTOCOL drive=")); Serial.print(Drives::driveName((DriveId)driveIndex));
  Serial.print(F(" result=")); Serial.print(stopped ? F("PASS") : F("FAIL"));
  Serial.print(F(" probe=")); Serial.print(probePct001);
  Serial.print(F(" comm701C=")); Serial.print(comm);
  Serial.print(F(" state703D=")); Serial.println(outAfter.runState703D);
  return stopped;
}
