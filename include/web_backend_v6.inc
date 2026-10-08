// Backend adapters keep the same web commands in FIELD and isolated SIM builds.
static ProgramLabelsV6 g_webLabels;
static bool g_webProgramEditing=false;
#if V6_DESKTOP_SIMULATION_ENABLED
static uint8_t g_simCalStage=0;
static uint32_t g_simCalStarted=0,g_simCalStopped=0;
static bool g_simCalSawRun=false;
#endif
static uint8_t webCalMask(){
#if V6_DESKTOP_SIMULATION_ENABLED
 uint8_t mask=0;for(uint8_t i=0;i<4;i++)if(g_simDirections.ready(i))mask|=1u<<i;return mask;
#else
 return g_he200Service.calibratedMask();
#endif
}
static int8_t webForwardSign(uint8_t i){
#if V6_DESKTOP_SIMULATION_ENABLED
 return g_simDirections.physicalSign(i,true);
#else
 return g_he200Service.forwardSensorSign(i);
#endif
}
static bool webBusBusy(){return !DESKTOP_SIMULATION_ENABLED&&(g_motor.vfdHasPendingWork()||g_motor.auditActive());}
static bool webMotionActive(){
#if V6_DESKTOP_SIMULATION_ENABLED
 return g_simPlant.moving();
#else
 return g_motor.isMotionActive();
#endif
}
static bool webPulseActive(){
#if V6_DESKTOP_SIMULATION_ENABLED
 return g_simCalStage||g_webCalPending;
#else
 return g_he200Service.pulseActive();
#endif
}
static uint16_t webSafetyState(){
#if V6_DESKTOP_SIMULATION_ENABLED
 return (g_simPlant.estop?1:0)|(g_simPlant.estopLatched?2:0);
#else
 return g_safety.stateWord();
#endif
}
static bool webSafetyBlocked(){return webSafetyState()!=0;}
static bool webClearSafety(){
#if V6_DESKTOP_SIMULATION_ENABLED
 return g_simPlant.clearLatch();
#else
 return g_safety.clearEstopLatch();
#endif
}
static uint32_t webAutoClock(uint32_t wall){
#if V6_DESKTOP_SIMULATION_ENABLED
 (void)wall;return g_simPlant.clock();
#else
 return wall;
#endif
}
static ModbusResult webReadParameter(uint8_t drive,uint16_t reg,uint16_t& value){
#if V6_DESKTOP_SIMULATION_ENABLED
 ModbusResult r{};r.simulated=true;r.ok=g_simPlant.read(drive,reg,value);r.error=r.ok?0:MODBUS_ERROR_TIMEOUT;return r;
#else
 return g_motor.readParameter(drive,reg,value);
#endif
}
static ModbusResult webWriteParameter(uint8_t drive,uint16_t reg,uint16_t value){
#if V6_DESKTOP_SIMULATION_ENABLED
 ModbusResult r{};r.simulated=true;r.ok=g_simPlant.write(drive,reg,value);r.error=r.ok?0:MODBUS_ERROR_WRITE_LOCKED;return r;
#else
 return g_motor.writeParameter(drive,reg,value);
#endif
}
static bool webSnapshot(uint8_t drive,He200ProtocolSnapshotV6& out){
#if V6_DESKTOP_SIMULATION_ENABLED
 if(!g_simPlant.connected(drive))return false;
 out.status3000=g_simPlant.frequency(drive)>20?1:3;
 out.runningFreq001Hz=g_simPlant.frequency(drive);out.runState703D=out.runningFreq001Hz>20?1:0;
 out.fault702D=(g_simPlant.faultMask&(1u<<drive))?1:0;return true;
#else
 return g_motor.protocolSnapshot(drive,out);
#endif
}
static void webResetDirections(){
#if V6_DESKTOP_SIMULATION_ENABLED
 g_simDirections.reset();
#else
 g_he200Service.resetGates();EEPROM.update(WEB_DIRECTION_EEPROM,0);
#endif
}
static bool webOutputTargets(const int16_t logical[4]){
#if V6_DESKTOP_SIMULATION_ENABLED
 for(uint8_t i=0;i<4;i++)if(logical[i]&&(!g_webLease.armed()||!webForwardSign(i)))return false;
 for(uint8_t i=0;i<4;i++){
  int16_t value=logical[i];const int16_t cap=g_settings.drive[i].maxPercent;
  if(value>cap)value=cap;
  if(value<-cap)value=-cap;
  g_simPlant.command(i,value*webForwardSign(i)*(g_settings.sensor[i].invert?-1:1));
 }
 return true;
#else
 return g_motor.requestServiceTargets(logical[0],logical[1],logical[2],logical[3]);
#endif
}
static void webReleaseOutput(){
#if V6_DESKTOP_SIMULATION_ENABLED
 g_simPlant.stop();
#else
 g_motor.serviceDecelStop(F("web jog released"));
#endif
}
static void webBackendStop(const __FlashStringHelper* why){
#if V6_DESKTOP_SIMULATION_ENABLED
 (void)why;g_simPlant.stop(true);g_simCalStage=0;
#else
 g_he200Service.stopAll(why);g_motor.stopAll(why);
#endif
}
static void webPrintLabel(uint8_t slot){
 const char* name=g_webLabels.name(slot);const char* hex="0123456789ABCDEF";
 Serial.print(F("@WEB_SLOT slot="));Serial.print(slot+1);Serial.print(F(" nameHex="));
 for(uint8_t i=0;name[i];i++){const uint8_t c=name[i];Serial.print(hex[c>>4]);Serial.print(hex[c&15]);}
 Serial.println();
}
#if V6_DESKTOP_SIMULATION_ENABLED
static void webModelDirections(){for(uint8_t i=0;i<4;i++)g_simDirections.restore(i,g_simPlant.forwardSign(i));}
static void webModelCalibration(uint32_t now){
 if(!g_webCalPending)return;
 const uint8_t drive=g_webCalPending-1;
 if(!g_simCalStage){
  if(g_simDirections.begin(drive,now)){
   g_simPlant.command(drive,g_webCalForward?g_webCalPct:-int16_t(g_webCalPct));
   g_simCalStage=1;g_simCalStarted=g_simPlant.clock();g_simCalSawRun=false;
  }
 }else if(g_simCalStage==1){
  if(g_simPlant.frequency(drive)>20)g_simCalSawRun=true;
  if(g_simPlant.clock()-g_simCalStarted>=g_webCalDuration){g_simPlant.stop();g_simCalStage=2;}
 }else if(g_simCalStage==2){
  if(g_simPlant.stopped()){g_simCalStopped=now;g_simCalStage=3;}
 }else if(g_simCalStage==3&&now-g_simCalStopped>=650){
  int32_t delta=0;const bool okay=g_simDirections.finish(drive,g_webCalForward,now,g_simCalStopped,g_simCalSawRun,delta);
  const char* names[]={"H1","H2","V1","V2"};
  Serial.print(F("@CAL drive="));Serial.print(names[drive]);Serial.print(F(" command="));Serial.print(g_webCalForward?F("FWD"):F("REV"));
  Serial.print(F(" deltaMm="));Serial.print(delta);Serial.print(F(" measured="));Serial.print(okay);
  Serial.print(F(" ready="));Serial.print(g_simDirections.ready(drive));Serial.print(F(" plusCommand="));Serial.print(webForwardSign(drive)>0?F("FWD"):F("REV"));Serial.println(F(" source=MODEL"));
  g_webCalPending=g_simCalStage=0;
 }
 if(g_webCalPending&&now-g_webCalPrepared>8000){g_simPlant.stop(true);g_simDirections.revoke(drive);g_webCalPending=g_simCalStage=0;Serial.println(F("@WEB_EVENT level=error code=CALIBRATION_TIMEOUT source=MODEL"));}
}
#endif
