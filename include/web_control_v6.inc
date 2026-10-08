// Included by main_v6_fast after its existing helpers; one owner of web motion.
#include <EEPROM.h>
#include "web_control_policy_v6.h"
#include "he200_parameters_v6.h"
#include "utils.h"
static WebLeaseV6 g_webLease;
static uint32_t g_webRequestId=0;
static uint32_t g_webSession=0,g_webJogMs=0,g_webStatusMs=0,g_webPollMs=0;
static uint32_t g_webQuietUntil=0;
static uint8_t g_webJogMask=0,g_webLastCalMask=0;
static bool g_webStepMode=false;
static uint16_t g_webLastStep=0;
static int16_t g_webJogTargets[4]={};
static uint8_t g_webCalPending=0,g_webCalPct=10;
static bool g_webCalForward=false;
static uint16_t g_webCalDuration=1200;
static uint32_t g_webCalPrepared=0;
struct WebDirectionRecordV6 { uint32_t magic; int8_t sign[4]; uint8_t p009[4]; uint16_t crc; };
struct WebBootRecordV6 { uint32_t magic,session;uint16_t crc; };
static constexpr int WEB_DIRECTION_EEPROM=4000;
static constexpr int WEB_BOOT_EEPROM=4032;
static_assert(512+sizeof(AutoProgramV6)*4<WEB_DIRECTION_EEPROM,"direction record must not overlap programs");

static void webError(const char* reason) { Serial.print(F("@WEB_ACK id="));Serial.print(g_webRequestId);Serial.print(F(" ok=0 reason="));Serial.println(reason); }
static void webOkay() { Serial.print(F("@WEB_ACK id="));Serial.print(g_webRequestId);Serial.println(F(" ok=1")); }
static void webProgram() {
 printProgramSummary();Serial.print(F("@WEB_ORDER values="));for(uint8_t i=0;i<g_program.zoneCount;i++){if(i)Serial.print(',');Serial.print(g_program.order[i]+1);}Serial.println();
 Serial.print(F("@WEB_DRY enabled="));Serial.print(g_program.dryingEnabled);
 Serial.print(F(" seconds="));Serial.print(g_program.dryingTimeS);Serial.print(F(" staging="));Serial.print(g_program.stagingZone+1);
 Serial.print(F(" x1="));Serial.print(g_program.dryX[0]);Serial.print(F(" x2="));Serial.print(g_program.dryX[1]);
 Serial.print(F(" z1="));Serial.print(g_program.dryZ[0]);Serial.print(F(" z2="));Serial.println(g_program.dryZ[1]);
}
static bool webFresh() {
 const uint32_t now=millis();for(uint8_t i=0;i<4;i++)
  if(!g_sensor[i].hwOk||!g_sensor[i].valid||!g_sensor[i].lastValidMs||now-g_sensor[i].lastValidMs>1200)return false;
 return true;
}
static bool webIdle() { return !g_auto.running()&&!g_webCalPending&&!g_webJogMask&&!g_motor.isMotionActive()&&!g_he200Service.pulseActive()&&!g_he200Service.assistActive()&&!g_motor.auditActive()&&!g_motor.vfdHasPendingWork(); }
static bool webStopped() {
 if(!webIdle())return false;
 for(uint8_t i=0;i<4;i++) { He200ProtocolSnapshotV6 s;
  if(!g_motor.protocolSnapshot(i,s)||s.status3000!=3||s.runState703D||s.runningFreq001Hz>20||s.fault702D)return false;
 }
 return true;
}
static void webStop(const __FlashStringHelper* why,bool disarm=true) {
 if(g_auto.running())g_auto.stop(why);
 g_he200Service.stopAll(why);g_motor.stopAll(why);
 g_webJogMask=0;g_webCalPending=0;g_webJogMs=0;g_webStepMode=false;g_systemMode=SystemModeV6::STOP;
 if(disarm){g_webLease.disarm();g_motor.webPermit(false);}
}
static void webStatus() {
 Serial.print(F("@WEB_STATE session="));Serial.print(g_webSession);
 Serial.print(F(" armed="));Serial.print(g_webLease.armed());
 Serial.print(F(" cal="));Serial.print(g_he200Service.calibratedMask());
 Serial.print(F(" busBusy="));Serial.print(g_motor.vfdHasPendingWork()||g_motor.auditActive());
 Serial.print(F(" jog="));Serial.print(g_webJogMask);
 Serial.print(F(" pulse="));Serial.print(g_he200Service.pulseActive()||g_webCalPending);
 Serial.print(F(" mode="));Serial.print((uint8_t)g_systemMode);
 Serial.print(F(" estop="));Serial.print(g_safety.stateWord());
 Serial.print(F(" limits="));Serial.print(g_safety.limitMask());
 Serial.print(F(" limitsEnabled="));Serial.print(g_safety.limitsEnabled());
 Serial.print(F(" externalSafetyRequired=1 running="));Serial.print(g_auto.running());
 Serial.print(F(" paused="));Serial.print(g_auto.paused());
 Serial.print(F(" waitOperator="));Serial.print(g_auto.waitOperator());
 Serial.print(F(" dryAlarm="));Serial.print(g_auto.dryAlarm());
 Serial.print(F(" sim="));Serial.print(g_auto.simulation());
 Serial.print(F(" phase="));Serial.print(g_auto.phaseName());
 Serial.print(F(" zone="));Serial.print(g_auto.zoneIndex()+1);
 Serial.print(F(" step="));Serial.print(g_auto.currentStep());
 Serial.print(F(" steps="));Serial.print(g_auto.totalSteps());
 Serial.print(F(" wait="));Serial.print(g_auto.remainingWaitSeconds(millis()));
 Serial.print(F(" error="));Serial.print(g_auto.error());
 Serial.print(F(" ready="));Serial.println(g_programStorage.readyForAuto(g_program));
 for(uint8_t i=0;i<4;i++) {
  Serial.print(F("@WEB_AXIS i="));Serial.print(i);
  Serial.print(F(" mm="));Serial.print(g_auto.simulation()&&g_auto.running()?g_auto.simPosition((SensorIndex)i):g_sensor[i].valueMm);
  Serial.print(F(" raw="));Serial.print(g_sensor[i].rawMm);
  Serial.print(F(" age="));Serial.print(g_sensor[i].lastValidMs?millis()-g_sensor[i].lastValidMs:99999UL);
  Serial.print(F(" fwdSign="));Serial.print(g_he200Service.forwardSensorSign(i));
  Serial.print(F(" max="));Serial.print(g_settings.drive[i].maxPercent);
  Serial.print(F(" manual="));Serial.print(g_settings.drive[i].manualPercent);
  Serial.print(F(" slow="));Serial.print(g_settings.drive[i].slowPercent);
  Serial.print(F(" band="));Serial.print(g_settings.drive[i].slowdownDistanceMm);
  Serial.print(F(" tolerance="));Serial.print(g_settings.drive[i].stopToleranceMm);
  Serial.print(F(" invert="));Serial.print(g_settings.sensor[i].invert);
  Serial.print(F(" offset="));Serial.println(g_settings.sensor[i].offsetMm);
 }
}
static bool webFrequencyOkay(uint8_t mask,uint8_t pct) {
 for(uint8_t i=0;i<4;i++)if(mask&(1u<<i)) {
  uint16_t maximum=0,start=0,low=0,res=0,mode=0;
  if(!g_motor.readParameter(i,0xF00A,maximum).ok||!g_motor.readParameter(i,0xF603,start).ok||
    !g_motor.readParameter(i,0xF00E,low).ok||!g_motor.readParameter(i,0xF016,res).ok||
    !g_motor.readParameter(i,0xF80E,mode).ok||res!=2)return false;
  uint32_t target=(uint32_t)maximum*pct/100;
  if(target<start || (target<low && mode!=0))return false;
  // Do not silently let the VFD raise a low test to its configured lower limit.
  if(target<low)return false;
 }
 return true;
}
static bool webProgramFrequencies() {
 for(uint8_t i=0;i<4;i++) {
  uint8_t pct=g_settings.drive[i].slowPercent;
  if(g_settings.drive[i].maxPercent<pct)pct=g_settings.drive[i].maxPercent;
  if(i>=2&&g_program.tiltPercent<pct)pct=g_program.tiltPercent;
  for(uint8_t j=0;j<g_program.zoneCount;j++)if(g_program.zones[j].enabled){
   const uint8_t cap=i<2?g_program.zones[j].movePercent:g_program.zones[j].verticalPercent;
   if(cap&&cap<pct)pct=cap;
  }
  if(!webFrequencyOkay(1u<<i,pct))return false;
 }
 return true;
}
static void webCoordinateSigns() {
 int8_t signs[4];for(uint8_t i=0;i<4;i++)signs[i]=g_he200Service.forwardSensorSign(i)*(g_settings.sensor[i].invert?-1:1);
 g_motor.coordinateSigns(signs);
}
static bool webToken(const char* text) { uint32_t n=0;return webUnsigned(text,0xffffffffUL,n)&&n==g_webSession; }
static void webSaveDirections() {
 WebDirectionRecordV6 r{};r.magic=0x57444331UL;
 for(uint8_t i=0;i<4;i++){r.sign[i]=g_he200Service.forwardSensorSign(i);uint16_t v=0;
  if(!g_motor.readParameter(i,0xF009,v).ok||v>1)return;
  r.p009[i]=v;}
 r.crc=crc16_modbus(reinterpret_cast<uint8_t*>(&r),offsetof(WebDirectionRecordV6,crc));
 EEPROM.put(WEB_DIRECTION_EEPROM,r);
 Serial.println(F("@WEB_CAL saved=1"));
}
static void webRestoreDirections() {
 WebDirectionRecordV6 r{};EEPROM.get(WEB_DIRECTION_EEPROM,r);
 if(r.magic!=0x57444331UL||r.crc!=crc16_modbus(reinterpret_cast<uint8_t*>(&r),offsetof(WebDirectionRecordV6,crc)))return;
 for(uint8_t i=0;i<4;i++) {uint16_t v=0;if(g_motor.readParameter(i,0xF009,v).ok&&v==r.p009[i])g_he200Service.restoreDirection(i,r.sign[i]);}
 webCoordinateSigns();
}
static bool webParseNumber(char* token,uint32_t max,uint32_t& value) { if(!webUnsigned(token,max,value)){webError("INVALID_NUMBER");return false;}return true; }
static bool webCommand(char* line) {
 if(strncmp(line,"web ",4))return false;
 if(!WEB_CONTROL_ENABLED){webError("WEB_BUILD_REQUIRED");return true;}
 char* t[18]={};uint8_t n=0;char* p=strtok(line," ");
 while(p&&n<18){t[n++]=p;p=strtok(nullptr," ");}
 if(p||n<2){webError("BAD_COMMAND");return true;}
 g_webRequestId=0;
 if(t[n-1][0]=='@') {if(!webUnsigned(t[n-1]+1,0xffffffffUL,g_webRequestId)){webError("REQUEST_ID");return true;}n--;}
 const char* cmd=t[1];
 if(!strcmp(cmd,"info")&&n==2){webStatus();webProgram();webOkay();return true;}
 if(!strcmp(cmd,"stop")&&n==2){webStop(F("web STOP"));webOkay();webStatus();return true;}
 if(!strcmp(cmd,"ping")&&n==3){if(webToken(t[2]))g_webLease.ping(millis());else webError("SESSION");return true;}
 if(!strcmp(cmd,"clear")&&n==3){if(webToken(t[2])&&webIdle()&&g_safety.clearEstopLatch()){webOkay();webStatus();}else webError("ESTOP_STILL_ACTIVE_OR_BUSY");return true;}
 if(!strcmp(cmd,"arm")&&n==3){
  if(!webToken(t[2])||g_safety.estopActive()||g_safety.estopLatched()||!webFresh()||!webStopped()){webError("ARM_REQUIRES_STOPPED_FRESH_SAFETY");return true;}
  g_webLease.arm(millis());g_motor.webPermit(true);g_he200Service.setPreflight(true,F("WEB_READBACK_STOPPED"));
  for(uint8_t i=0;i<4;i++)g_he200Service.markProtocol(i,true);
  webRestoreDirections();webOkay();webStatus();return true;
 }
 if(!strcmp(cmd,"read")&&n==4){
  g_webQuietUntil=millis()+600;
  const int8_t i=serviceDriveIndex(t[2]);He200ParameterV6 parameter;uint16_t value=0;
  if(!webIdle()||i<0||!he200WebParameter(t[3],parameter)){webError("READ_BUSY_OR_UNKNOWN");return true;}
  const ModbusResult r=g_motor.readParameter(i,parameter.reg,value);
  Serial.print(F("@WEB_PARAM id="));Serial.print(g_webRequestId);Serial.print(F(" drive="));Serial.print(t[2]);Serial.print(F(" key="));Serial.print(parameter.key);
  Serial.print(F(" ok="));Serial.print(r.ok);Serial.print(F(" raw="));Serial.print(value);
  Serial.print(F(" error="));Serial.print(r.error);Serial.print(F(" exception="));Serial.println(r.exception);return true;
 }
 if(!g_webLease.armed()||g_webLease.expired(millis())||g_safety.estopActive()||g_safety.estopLatched()) {webError("NOT_ARMED_OR_SAFETY");return true;}
 if(n<3||!webToken(t[2])){webError("SESSION");return true;}
 g_webLease.ping(millis());
 if(!strcmp(cmd,"write")&&n==7){
  const int8_t i=serviceDriveIndex(t[3]);He200ParameterV6 parameter;uint32_t before=0,target=0;
  if(i<0||!he200WebParameter(t[4],parameter)||!parameter.writable||!webParseNumber(t[5],65535,before)||!webParseNumber(t[6],65535,target)||target<parameter.min||target>parameter.max||!webStopped()){webError("WRITE_REQUIRES_STOPPED_ALLOWED_RANGE");return true;}
  uint16_t old=0,after=0;
  if(!g_motor.readParameter(i,parameter.reg,old).ok||old!=before){webError("VALUE_CHANGED_RELOAD");return true;}
  const ModbusResult r=g_motor.writeParameter(i,parameter.reg,target);
  const bool verified=r.ok&&g_motor.readParameter(i,parameter.reg,after).ok&&after==target;
  Serial.print(F("@WEB_WRITE drive="));Serial.print(t[3]);Serial.print(F(" key="));Serial.print(t[4]);
  Serial.print(F(" before="));Serial.print(old);Serial.print(F(" requested="));Serial.print(target);Serial.print(F(" after="));Serial.print(after);Serial.print(F(" ok="));Serial.println(verified);
  if(parameter.reg==0xF009){g_he200Service.resetGates();webCoordinateSigns();EEPROM.update(WEB_DIRECTION_EEPROM,0);webStop(F("direction parameter changed"));}
  if(!verified){webStop(F("parameter verify failed"));webError("WRITE_VERIFY_FAILED");}else {g_webLease.ping(millis());webOkay();}return true;
 }
 if(!strcmp(cmd,"calibrate")&&n==7){
  const int8_t i=serviceDriveIndex(t[3]);uint32_t pct=0,ms=0;
  if(i<0||(!strcmp(t[4],"fwd")==!strcmp(t[4],"rev"))||!webParseNumber(t[5],20,pct)||pct<5||!webParseNumber(t[6],3000,ms)||ms<500||!webStopped()||!webFrequencyOkay(1u<<i,pct)){webError("CALIBRATION_INPUT_OR_FREQUENCY");return true;}
  serviceSensorsFast();
  if(pct>g_settings.drive[i].maxPercent){webError("CALIBRATION_PROFILE_MAX");return true;}
  g_webLease.ping(millis());
  g_webCalPending=i+1;g_webCalForward=!strcmp(t[4],"fwd");g_webCalPct=pct;g_webCalDuration=ms;g_webCalPrepared=millis();webOkay();webStatus();return true;
 }
 if(!strcmp(cmd,"jog")&&n==6){
  const uint8_t mask=serviceDriveMask(t[3]);bool positive=false;uint32_t pct=0;
  if(!mask||!serviceDirectionPositive(t[4],positive)||!webParseNumber(t[5],80,pct)||pct<5||g_auto.running()||g_webCalPending||g_he200Service.pulseActive()||g_motor.auditActive()||!webFresh()||(g_he200Service.calibratedMask()&mask)!=mask||(g_webJogMask&&g_webJogMask!=mask)){webError("JOG_GATES");return true;}
  if(!g_webJogMask&&!webStopped()){webError("JOG_REQUIRES_STOPPED");return true;}
  if(!g_webJogMask && !webFrequencyOkay(mask,pct)){webError("JOG_FREQUENCY");return true;}
  if(g_webJogMask)for(uint8_t i=0;i<4;i++)if((mask&(1u<<i))&&g_webJogTargets[i]!=(positive?int16_t(pct):-int16_t(pct))){webError("RELEASE_BEFORE_DIRECTION_CHANGE");return true;}
  for(uint8_t i=0;i<4;i++)if((mask&(1u<<i))&&pct>g_settings.drive[i].maxPercent){webError("JOG_PROFILE_MAX");return true;}
  serviceSensorsFast();if(!webFresh()){webError("SENSORS");return true;}g_webLease.ping(millis());
  for(uint8_t i=0;i<4;i++)g_webJogTargets[i]=(mask&(1u<<i))?(positive?pct:-int16_t(pct)):0;
  webCoordinateSigns();g_systemMode=SystemModeV6::MANUAL;
  if(!g_motor.requestServiceTargets(g_webJogTargets[0],g_webJogTargets[1],g_webJogTargets[2],g_webJogTargets[3]))webError("JOG_OUTPUT");
  else {g_webJogMask=mask;g_webJogMs=millis();webOkay();}return true;
 }
 if(!strcmp(cmd,"release")&&n==3){g_webJogMask=0;g_motor.serviceDecelStop(F("web jog released"));webOkay();return true;}
 if(!strcmp(cmd,"start")&&n==4){
  if(!webStopped()||g_he200Service.calibratedMask()!=15||!webFresh()||(!strcmp(t[3],"auto")==!strcmp(t[3],"steps"))){webError("AUTO_GATES");return true;}
  if(!webProgramFrequencies()){webError("PROGRAM_SPEED_BELOW_START_OR_LOWER_LIMIT");return true;}
  serviceSensorsFast();if(!webFresh()){webError("SENSORS");return true;}g_webLease.ping(millis());
  g_systemMode=SystemModeV6::AUTO;g_webStepMode=!strcmp(t[3],"steps");g_webLastStep=0;
  if(!g_auto.start(g_program,buildAutoSensors(),false,millis()))webError("AUTO_PROGRAM");else webOkay();return true;
 }
 if(!strcmp(cmd,"home")&&n==3){
  if(!webStopped()||g_he200Service.calibratedMask()!=15||!webFresh()){webError("HOME_GATES");return true;}
  if(!webProgramFrequencies()){webError("HOME_FREQUENCY");return true;}serviceSensorsFast();if(!webFresh()){webError("SENSORS");return true;}g_webLease.ping(millis());
  g_systemMode=SystemModeV6::HOME;if(!g_auto.startHome(g_program,buildAutoSensors(),false,millis()))webError("HOME_PROGRAM");else webOkay();return true;
 }
 if(!strcmp(cmd,"pause")&&n==3){g_auto.pause();webOkay();return true;}
 if(!strcmp(cmd,"resume")&&n==3){if(!webFresh()||g_he200Service.calibratedMask()!=15){webError("RESUME_GATES");return true;}g_auto.resume(millis());webOkay();return true;}
 if(!strcmp(cmd,"next")&&n==3){if(g_auto.paused())g_auto.resume(millis());else g_auto.operatorNext();webOkay();return true;}
 if(!strcmp(cmd,"forget")&&n==3&&webIdle()){g_he200Service.resetGates();webCoordinateSigns();EEPROM.update(WEB_DIRECTION_EEPROM,0);webStop(F("calibration reset"));webOkay();return true;}
 // Editing requires stopped control; no program or coordinate mutation in flight.
 if(!webIdle()){webError("EDIT_REQUIRES_IDLE");return true;}
 if(!strcmp(cmd,"program")&&n>=4){
  if(!strcmp(t[3],"show")&&n==4){webProgram();webOkay();return true;}
  if(!strcmp(t[3],"save")&&n==4){if(!saveProgramSlotV6())webError("SAVE_PROGRAM");else webOkay();return true;}
  if(!strcmp(t[3],"defaults")&&n==4){g_programStorage.defaults(g_program,g_programSlot);markProgramDirty();webOkay();return true;}
  uint32_t a=0,b=0;
  if(!strcmp(t[3],"count")&&n==5&&webParseNumber(t[4],10,a)&&a>=1){g_program.zoneCount=a;g_program.validMask=0;for(uint8_t i=0;i<a;i++){g_program.order[i]=i;g_program.zones[i].validMask=0;}markProgramDirty();webOkay();return true;}
  if(!strcmp(t[3],"slot")&&n==5&&webParseNumber(t[4],4,a)&&a>=1){if(!loadProgramSlotV6(a-1))webError("EMPTY_OR_INVALID_SLOT");else webOkay();return true;}
  if((!strcmp(t[3],"home")||!strcmp(t[3],"travel"))&&n==6&&webParseNumber(t[4],10000,a)&&webParseNumber(t[5],10000,b)){
   int32_t* pair=!strcmp(t[3],"home")?g_program.homeX:g_program.travelZ;pair[0]=a;pair[1]=b;
   g_program.validMask|=!strcmp(t[3],"home")?AUTO_PROGRAM_VALID_HOME_X:AUTO_PROGRAM_VALID_TRAVEL_Z;markProgramDirty();webOkay();return true;
  }
  if(!strcmp(t[3],"capture")&&n==5&&webFresh()){
   if(!strcmp(t[4],"home")){g_program.homeX[0]=g_sensor[0].valueMm;g_program.homeX[1]=g_sensor[1].valueMm;g_program.validMask|=AUTO_PROGRAM_VALID_HOME_X;}
   else if(!strcmp(t[4],"travel")){g_program.travelZ[0]=g_sensor[2].valueMm;g_program.travelZ[1]=g_sensor[3].valueMm;g_program.validMask|=AUTO_PROGRAM_VALID_TRAVEL_Z;}
   else {webError("CAPTURE_TARGET");return true;}markProgramDirty();webOkay();return true;
  }
  if(!strcmp(t[3],"recipe")&&n==7&&webParseNumber(t[4],36000,a)&&webParseNumber(t[5],80,b)){
   uint32_t side=0;if(!webParseNumber(t[6],1,side)||b<5)return true;
   g_program.dripWaitS=a;g_program.tiltPercent=b;g_program.lowSide=side;markProgramDirty();webOkay();return true;
  }
  if(!strcmp(t[3],"dry")&&n==11){
   uint32_t v[7],max[7]={1,36000,g_program.zoneCount,10000,10000,10000,10000};
   for(uint8_t i=0;i<7;i++)if(!webParseNumber(t[i+4],max[i],v[i]))return true;
   if(!v[2]){webError("STAGING_ZONE");return true;}
   g_program.dryingEnabled=v[0];g_program.dryingTimeS=v[1];g_program.stagingZone=v[2]-1;
   g_program.dryX[0]=v[3];g_program.dryX[1]=v[4];g_program.dryZ[0]=v[5];g_program.dryZ[1]=v[6];
   g_program.dryValid=1;g_program.validMask|=AUTO_PROGRAM_VALID_DRY_X|AUTO_PROGRAM_VALID_DRY_Z;markProgramDirty();webOkay();return true;
  }
  if(!strcmp(t[3],"order")&&n==4+g_program.zoneCount){
   uint16_t mask=0;uint8_t order[10];for(uint8_t i=0;i<g_program.zoneCount;i++){if(!webParseNumber(t[i+4],g_program.zoneCount,a)||a<1||(mask&(1u<<(a-1)))){webError("ORDER");return true;}mask|=1u<<(a-1);order[i]=a-1;}
   memcpy(g_program.order,order,g_program.zoneCount);markProgramDirty();webOkay();return true;
  }
 }
 if(!strcmp(cmd,"zone")&&n==14){
  uint32_t v[11];const uint32_t max[11]={10,10000,10000,10000,10000,36000,5000,36000,80,80,1};
  for(uint8_t i=0;i<11;i++)if(!webParseNumber(t[i+3],max[i],v[i]))return true;
  if(!v[0]||v[8]<5||v[9]<5){webError("ZONE_RANGE");return true;}
  if(v[0]>g_program.zoneCount){for(uint8_t i=g_program.zoneCount;i<v[0];i++)g_program.order[i]=i;g_program.zoneCount=v[0];}
  AutoZoneV6& z=g_program.zones[v[0]-1];z.xMm[0]=v[1];z.xMm[1]=v[2];z.zMm[0]=v[3];z.zMm[1]=v[4];z.dipTimeS=v[5];z.tiltStepMm=v[6];z.stepWaitS=v[7];z.movePercent=v[8];z.verticalPercent=v[9];z.enabled=v[10];z.validMask=AUTO_ZONE_VALID_ALL;markProgramDirty();webOkay();return true;
 }
 if(!strcmp(cmd,"testplan")&&n==6&&webFresh()){
  uint32_t dx=0,dz=0,wait=0;if(!webParseNumber(t[3],500,dx)||!webParseNumber(t[4],500,dz)||!webParseNumber(t[5],60,wait))return true;
  for(uint8_t i=0;i<2;i++)if(g_sensor[i].valueMm+dx>10000||g_sensor[i+2].valueMm<int32_t(dz)){webError("TEST_PLAN_RANGE");return true;}
  g_programStorage.defaults(g_program,g_programSlot);g_program.zoneCount=2;g_program.validMask=AUTO_PROGRAM_VALID_HOME_X|AUTO_PROGRAM_VALID_TRAVEL_Z;g_program.dripWaitS=wait;g_program.tiltPercent=10;
  for(uint8_t i=0;i<2;i++){g_program.homeX[i]=g_sensor[i].valueMm;g_program.travelZ[i]=g_sensor[i+2].valueMm;}
  for(uint8_t zi=0;zi<2;zi++){auto& z=g_program.zones[zi];for(uint8_t i=0;i<2;i++){z.xMm[i]=g_program.homeX[i]+(zi?dx:0);z.zMm[i]=g_program.travelZ[i]-dz;}z.validMask=3;z.enabled=1;z.dipTimeS=wait;z.tiltStepMm=0;z.stepWaitS=0;z.movePercent=10;z.verticalPercent=10;}
  markProgramDirty();printProgramSummary();webOkay();return true;
 }
 if(!strcmp(cmd,"profile")&&n==9){
  int8_t i=serviceDriveIndex(t[3]);uint32_t v[5],max[5]={80,80,80,3000,100};
  if(i<0){webError("DRIVE");return true;}for(uint8_t j=0;j<5;j++)if(!webParseNumber(t[j+4],max[j],v[j]))return true;
  if(v[0]<5||v[1]<5||v[2]<5||v[0]>v[1]||v[2]>v[1]||!v[4]){webError("PROFILE");return true;}
  auto& d=g_settings.drive[i];d.manualPercent=v[0];d.maxPercent=v[1];d.slowPercent=v[2];d.slowdownDistanceMm=v[3];d.stopToleranceMm=v[4];g_motor.applySettings(g_settings);g_settingsDirty=true;webOkay();return true;
 }
 if(!strcmp(cmd,"settings-save")&&n==3){g_storage.save(g_settings);g_settingsDirty=false;webOkay();return true;}
 if(!strcmp(cmd,"sensor")&&n==6){
  int8_t i=serviceDriveIndex(t[3]);uint32_t inverse=0,offset=0;const bool negative=t[5][0]=='-';
  if(i<0||!webParseNumber(t[4],1,inverse)||!webParseNumber(t[5]+(negative?1:0),10000,offset)){webError("SENSOR_SETTINGS");return true;}
  g_settings.sensor[i].invert=inverse;g_settings.sensor[i].offsetMm=negative?-int16_t(offset):int16_t(offset);
  g_program.validMask=0;for(uint8_t j=0;j<g_program.zoneCount;j++)g_program.zones[j].validMask=0;
  markProgramDirty();g_settingsDirty=true;webCoordinateSigns();webOkay();return true;
 }
 webError("BAD_COMMAND_OR_RANGE");return true;
}
static void webBegin() {
 if(!WEB_CONTROL_ENABLED)return;
 WebBootRecordV6 record{};EEPROM.get(WEB_BOOT_EEPROM,record);
 if(record.magic!=0x57425331UL||record.crc!=crc16_modbus(reinterpret_cast<uint8_t*>(&record),offsetof(WebBootRecordV6,crc)))record.session=0;
 record.magic=0x57425331UL;if(++record.session==0)record.session=1;
 record.crc=crc16_modbus(reinterpret_cast<uint8_t*>(&record),offsetof(WebBootRecordV6,crc));EEPROM.put(WEB_BOOT_EEPROM,record);
 g_webSession=record.session;g_webLease.disarm();g_motor.webPermit(false);g_consolePeriodicEnabled=false;webStatus();
}
static void webService(uint32_t now) {
 if(!WEB_CONTROL_ENABLED)return;
 webCoordinateSigns();
 if(g_webLease.expired(now)||((g_webJogMask||g_auto.running()||g_he200Service.pulseActive())&&!webFresh())||g_safety.estopActive()||g_safety.estopLatched()) {
  if(g_webLease.armed()||g_webJogMask||g_webCalPending||g_auto.running()||g_he200Service.pulseActive())webStop(F("web lease/sensor/safety"));
 }
 if(g_webCalPending){
  if(now-g_webCalPrepared>3000){webError("CALIBRATION_PREPARE_TIMEOUT");webStop(F("calibration sensors"));}
  else if(g_he200Service.calibrationSamplesReady(now)){
   const uint8_t drive=g_webCalPending-1;g_webCalPending=0;
   if(!g_he200Service.startPulse(1u<<drive,g_webCalForward,g_webCalPct,g_webCalDuration,now,true))webError("CALIBRATION_REJECTED");
  }
 }
 if(g_webJogMask && now-g_webJogMs>400){g_webJogMask=0;g_motor.serviceDecelStop(F("web deadman expired"));}
 if(g_webJogMask||g_auto.running()||g_he200Service.pulseActive())for(uint8_t i=0;i<4;i++){
  const auto& telemetry=g_motor.vfdTelemetry(i);
  if(!telemetry.connected||!telemetry.lastOkMs||now-telemetry.lastOkMs>5000||telemetry.faultCode){webStop(F("VFD telemetry lost/fault"));webError("VFD_HEALTH");break;}
 }
 if(g_webJogMask && g_safety.limitsEnabled()&&g_safety.limitMask())webStop(F("controller limit"));
 const uint8_t mask=g_he200Service.calibratedMask();
 if(mask!=g_webLastCalMask&&!g_he200Service.pulseActive()&&webIdle()){g_webLastCalMask=mask;if(mask)webSaveDirections();}
 if(g_webStepMode&&g_auto.running()&&!g_auto.paused()&&g_auto.currentStep()!=g_webLastStep){
  if(g_webLastStep)g_auto.pause();
  g_webLastStep=g_auto.currentStep();
 }
 if(now-g_webPollMs>1500&&(int32_t)(now-g_webQuietUntil)>=0&&!g_motor.auditActive()&&!g_he200Service.pulseActive()){g_webPollMs=now;g_motor.testVfdConnection(255);}
 if(now-g_webStatusMs>500){g_webStatusMs=now;webStatus();}
}
