#include "Arduino.h"
#include "EEPROM.h"
#include "config_v6_bringup.h"
#include "program_v6.h"
#include "web_control_policy_v6.h"
#include "he200_parameters_v6.h"
#include "settings_v6.h"
#include "modbus.h"
#include "system_state.h"
#include <assert.h>
uint32_t testClock=1000;
HardwareSerial Serial;
TestEEPROMV6 EEPROM;
uint16_t crc16_modbus(const uint8_t* p,size_t n){uint16_t c=65535;while(n--){c^=*p++;for(int i=0;i<8;i++)c=(c&1)?(c>>1)^0xA001:c>>1;}return c;}
struct He200ProtocolSnapshotV6 {uint16_t status3000=3,runState703D=0,runningFreq001Hz=0,fault702D=0;};
struct Sensor {bool hwOk=true,valid=true;uint32_t lastValidMs=1000;int32_t rawMm=1000,valueMm=1000;};
static Sensor g_sensor[4];
static SettingsV6 g_settings;
static AutoProgramV6 g_program;
static ProgramStorageV6 g_programStorage;
static uint8_t g_programSlot=0;
static bool g_programDirty=false,g_settingsDirty=false,g_consolePeriodicEnabled=true;
static SystemModeV6 g_systemMode=SystemModeV6::SERVICE;
struct FakeSafety {bool active=false,latched=false;bool clearEstopLatch(){if(active)return false;latched=false;return true;}bool estopActive(){return active;}bool estopLatched(){return latched;}uint8_t limitMask(){return 0;}bool limitsEnabled(){return false;}uint16_t stateWord(){return active?1:0;}};
static FakeSafety g_safety;
struct FakeAuto {
 bool run=false,pauseFlag=false;unsigned starts=0;
 bool running(){return run;}bool paused(){return pauseFlag;}bool simulation(){return false;}bool waitOperator(){return false;}bool dryAlarm(){return false;}
 void stop(const __FlashStringHelper*){run=false;}void pause(){pauseFlag=true;}void resume(uint32_t){pauseFlag=false;}void operatorNext(){}
 const __FlashStringHelper* phaseName(){return F("IDLE");}uint8_t zoneIndex(){return 0;}uint16_t currentStep(){return 1;}uint16_t totalSteps(){return 3;}uint16_t remainingWaitSeconds(uint32_t){return 0;}uint16_t error(){return 0;}
 int32_t simPosition(SensorIndex){return 0;}
 bool start(const AutoProgramV6&,int,bool,uint32_t){starts++;run=true;return true;}bool startHome(const AutoProgramV6&,int,bool,uint32_t){starts++;run=true;return true;}
};static FakeAuto g_auto;
struct FakeService {
 uint8_t mask=0;bool pulse=false;unsigned pulses=0;
 void stopAll(const __FlashStringHelper*){pulse=false;}bool pulseActive(){return pulse;}bool assistActive(){return false;}bool calibrationSamplesReady(uint32_t){return false;}
 uint8_t calibratedMask(){return mask;}int8_t forwardSensorSign(uint8_t i){return mask&(1u<<i)?-1:0;}void restoreDirection(uint8_t i,int8_t sign){if(sign==1||sign==-1)mask|=1u<<i;}
 void resetGates(){mask=0;}void setPreflight(bool,const __FlashStringHelper*){}void markProtocol(uint8_t,bool){}
 bool startPulse(uint8_t,bool,uint8_t,uint16_t,uint32_t,bool){pulses++;pulse=true;return true;}
};static FakeService g_he200Service;
struct FakeMotor {
 struct Telemetry {bool connected=true;uint32_t lastOkMs=1000;uint16_t faultCode=0;} telemetry;
 const Telemetry& vfdTelemetry(uint8_t){return telemetry;}
 bool pending=false,audit=false,permit=false,stopped=true;unsigned writes=0,motions=0,stops=0;uint16_t start=0,lower=0,stored=0;
 bool auditActive(){return audit;}bool isMotionActive(){return motions>0;}bool vfdHasPendingWork(){return pending;}
 void stopAll(const __FlashStringHelper*){stops++;motions=0;}void serviceDecelStop(const __FlashStringHelper*){stops++;motions=0;}
 void webPermit(bool b){permit=b;}void coordinateSigns(const int8_t*){}void testVfdConnection(uint8_t){}void applySettings(const SettingsV6&){}
 bool protocolSnapshot(uint8_t,He200ProtocolSnapshotV6& s){s.status3000=stopped?3:1;return true;}
 ModbusResult readParameter(uint8_t,uint16_t reg,uint16_t& value){value=reg==0xF00A?5000:reg==0xF603?start:reg==0xF00E?lower:reg==0xF016?2:reg==0xF009?0:reg==0xF604?stored:0;return {true,0,false,0};}
 ModbusResult writeParameter(uint8_t,uint16_t,uint16_t value){writes++;stored=value;return {true,0,false,0};}
 bool requestServiceTargets(int16_t,int16_t,int16_t,int16_t){motions++;return true;}
};static FakeMotor g_motor;
struct FakeStorage {void save(SettingsV6&){}};static FakeStorage g_storage;
static void printProgramSummary(){}static void markProgramDirty(){g_programDirty=true;}
static bool saveProgramSlotV6(){return true;}static bool loadProgramSlotV6(uint8_t){return true;}
static int buildAutoSensors(){return 0;}static void serviceSensorsFast(){for(auto& s:g_sensor)s.lastValidMs=testClock;}
static int8_t serviceDriveIndex(const char* s){const char* names[]={"h1","h2","v1","v2"};for(uint8_t i=0;i<4;i++)if(!strcmp(s,names[i]))return i;return -1;}
static uint8_t serviceDriveMask(const char* s){int8_t i=serviceDriveIndex(s);return i>=0?1u<<i:!strcmp(s,"h")?3:!strcmp(s,"v")?12:0;}
static bool serviceDirectionPositive(const char* s,bool& p){p=!strcmp(s,"pos");return p||!strcmp(s,"neg");}
#include "web_control_v6.inc"
static void command(const char* text){char line[160];strncpy(line,text,sizeof(line));line[159]=0;assert(webCommand(line));}
int main(){
 for(auto& p:g_settings.drive)p={10,40,10,0,100,8};g_programStorage.defaults(g_program,0);webBegin();assert(!g_webLease.armed()&&!g_motor.permit);const uint32_t previousSession=g_webSession;webBegin();assert(g_webSession!=previousSession&&!g_webLease.armed());g_webSession=42;
 command("web jog 42 h1 pos 10 @1");assert(!g_motor.motions);
 command("web arm 43 @2");assert(!g_webLease.armed());
 command("web arm 42 @3");assert(g_webLease.armed()&&g_motor.permit);
 command("web start 42 auto @4");assert(!g_auto.starts);
 command("web jog 42 h1 pos 10 @5");assert(!g_motor.motions);
 g_he200Service.mask=15;command("web jog 42 h1 undefined 10 @6");assert(!g_motor.motions);
 g_motor.start=1000;command("web jog 42 h1 pos 10 @7");assert(!g_motor.motions);
 g_motor.start=0;command("web jog 42 h1 pos 10 @8");assert(g_motor.motions==1&&g_webJogMask==1);
 command("web write 42 h1 P6.04 0 3 @9");assert(g_motor.writes==0);
 command("web release 42 @10");assert(g_webJogMask==0);
 command("web write 42 h1 P4.02 10 0 @11");assert(!g_motor.writes);
 command("web write 42 h1 0x2000 0 1 @12");assert(!g_motor.writes);
 command("web write 42 h1 P6.04 0 1001 @13");assert(!g_motor.writes);
 command("web write 42 h1 P6.04 1 3 @14");assert(!g_motor.writes);
 command("web write 42 h1 P6.04 0 3 @15");assert(g_motor.writes==1&&g_motor.stored==3);
 command("web zone 42 1 1000 1000 900 900 36000 0 36000 10 10 1 @16");assert(g_program.zones[0].dipTimeS==36000&&g_program.zones[0].stepWaitS==36000);
 command("web program 42 count 2 @17");assert(g_program.zoneCount==2&&!g_program.validMask&&!g_program.zones[0].validMask);
 command("web program 42 order 1 1 @18");assert(g_program.order[1]==1);
 command("web program 42 dry 1 60 2 1500 1500 900 900 @180");assert(g_program.dryingEnabled&&g_program.dryValid&&g_program.dryingTimeS==60&&g_program.stagingZone==1);
 g_motor.stopped=false;command("web start 42 auto @181");assert(!g_auto.starts);command("web home 42 @182");assert(!g_auto.starts);g_motor.stopped=true;
 command("web stop @19");assert(!g_webLease.armed()&&!g_motor.permit&&!g_auto.run);
 command("web arm 42 @20");assert(g_webLease.armed());testClock+=1600;webService(testClock);assert(!g_webLease.armed()&&!g_motor.permit);
 command("web arm 42 @21"); // old samples correctly reject a new arm
 assert(!g_webLease.armed());serviceSensorsFast();command("web arm 42 @22");assert(g_webLease.armed());
 g_safety.active=true;webService(testClock);assert(!g_webLease.armed());
 g_safety.latched=true;command("web clear 42 @23");assert(g_safety.latched);
 g_safety.active=false;command("web clear 42 @24");assert(!g_safety.latched);
 serviceSensorsFast();command("web arm 42 @25");assert(g_webLease.armed());g_webJogMask=1;g_webJogMs=testClock;g_motor.telemetry.connected=false;webService(testClock);assert(!g_webLease.armed()&&!g_webJogMask);
 puts("PASS: actual web command parser with mock I/O; session/arm/calibration/frequency gates; jog/release; motion blocks writes; allowlist/range/compare-and-set; uint16 dwell; atomic readiness reset; STOP/deadman/E-STOP");
}
