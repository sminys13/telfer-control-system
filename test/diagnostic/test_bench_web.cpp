#include "Arduino.h"
#include "EEPROM.h"
#include "config_v6_bringup.h"
#include "auto_runner_v6.h"
#include "simulation_plant_v6.h"
#include "direction_calibration_v6.h"
#include "modbus.h"
#include <assert.h>
uint32_t testClock=1000;
HardwareSerial Serial;
TestEEPROMV6 EEPROM;
unsigned physicalCalls=0;
ModbusMasterRTU::ModbusMasterRTU(){}
bool MotorControlV6::requestAutoTargets(SystemModeV6,int16_t,int16_t,int16_t,int16_t){physicalCalls++;return false;}
void MotorControlV6::applySettings(const SettingsV6&){physicalCalls++;}
void MotorControlV6::autoStop(const __FlashStringHelper*){physicalCalls++;}
uint16_t crc16_modbus(const uint8_t* p,size_t n){uint16_t c=65535;while(n--){c^=*p++;for(int i=0;i<8;i++)c=(c&1)?(c>>1)^0xA001:c>>1;}return c;}
struct Sensor {bool hwOk=true,valid=true;uint32_t lastValidMs=1000;int32_t rawMm=1000,valueMm=1000;};
static Sensor g_sensor[4];
static SettingsV6 g_settings;
static SettingsStorageV6 g_storage;
static AutoProgramV6 g_program;
static ProgramStorageV6 g_programStorage;
static uint8_t g_programSlot=0;
static bool g_programDirty=false,g_settingsDirty=false,g_consolePeriodicEnabled=true;
static SystemModeV6 g_systemMode=SystemModeV6::SERVICE;
static MotorControlV6 g_motor;
static AutoRunnerV6 g_auto;
static SimulationPlantV6 g_simPlant;
static DirectionCalibrationV6 g_simDirections;
struct FakeSafety {bool limitsEnabled(){return false;}uint8_t limitMask(){return 0;}};static FakeSafety g_safety;
struct FakeService {bool assistActive(){return false;}bool pulseActive(){return false;}void setPreflight(bool,const __FlashStringHelper*){}void markProtocol(uint8_t,bool){}void restoreDirection(uint8_t,int8_t){assert(false);}};static FakeService g_he200Service;
static void printProgramSummary(){}
static void markProgramDirty(){g_programDirty=true;}
static bool saveProgramSlotV6(){return g_programStorage.save(g_programSlot,g_program);}
static bool loadProgramSlotV6(uint8_t i){g_programSlot=i;if(g_programStorage.load(i,g_program))return true;g_programStorage.defaults(g_program,i);return false;}
static AutoSensorsV6 buildAutoSensors(){AutoSensorsV6 s;for(uint8_t i=0;i<4;i++){s.mm[i]=g_sensor[i].valueMm;if(g_sensor[i].valid)s.usableMask|=1u<<i;}return s;}
static void serviceSensorsFast(){g_simPlant.tick(testClock);if(g_simPlant.sampleDue(testClock))for(uint8_t i=0;i<4;i++){
 auto& s=g_sensor[i];s.valid=!(g_simPlant.sensorLostMask&(1u<<i));if(s.valid){s.rawMm=g_simPlant.raw(i);s.valueMm=g_storage.applyCalibration(g_settings,(SensorIndex)i,s.rawMm);s.lastValidMs=testClock;g_simDirections.samples[i].add(s.rawMm,testClock);}else g_simDirections.samples[i].add(-1,testClock);
}}
static int8_t serviceDriveIndex(const char* s){const char* names[]={"h1","h2","v1","v2"};for(uint8_t i=0;i<4;i++)if(!strcmp(s,names[i]))return i;return -1;}
static uint8_t serviceDriveMask(const char* s){int8_t i=serviceDriveIndex(s);return i>=0?1u<<i:!strcmp(s,"h")?3:!strcmp(s,"v")?12:0;}
static bool serviceDirectionPositive(const char* s,bool& p){p=!strcmp(s,"pos");return p||!strcmp(s,"neg");}
#include "web_control_v6.inc"
static void command(const char* s){char line[160];strncpy(line,s,sizeof(line));line[159]=0;assert(webCommand(line));}
static void tick(unsigned count,bool heartbeat=true){for(unsigned j=0;j<count;j++){
 testClock+=20;serviceSensorsFast();if(heartbeat&&j%8==0)command("web ping 42");webService(testClock);
 if(g_auto.running()){g_auto.service(webAutoClock(testClock),buildAutoSensors(),webSafetyBlocked(),0);int16_t t[4];for(uint8_t i=0;i<4;i++)t[i]=g_auto.simulationTarget(i);assert(webOutputTargets(t));}
}}
int main(){
 static_assert(DESKTOP_SIMULATION_ENABLED&&!VFD_RS485_ENABLED&&!VFD_WRITE_COMMANDS_ENABLED&&!AUTO_PHYSICAL_ENABLED&&HE200_DIAGNOSTIC_LOCK,"isolated build required");
 memset(EEPROM.bytes,0x5A,1856);g_storage.defaults(g_settings);for(auto& d:g_settings.drive)d={10,40,10,0,100,8};
 g_programStorage.defaults(g_program,0);g_auto.begin(g_motor,g_settings);g_auto.externalSimulation(true);g_simPlant.begin(testClock);webModelDirections();webBegin();g_webSession=42;tick(15,false);
 command("web arm 42");assert(g_webLease.armed());command("web jog 42 h1 pos 10");assert(g_webJogMask==1);
 const int32_t start=g_simPlant.raw(0);tick(10);command("web jog 42 h1 pos 10");tick(10);assert(g_simPlant.raw(0)>start);
 command("web release 42");tick(15);assert(g_simPlant.stopped());
 command("web forget 42");tick(20,false);command("web arm 42");command("web calibrate 42 h1 fwd 10 1200");tick(200);assert(!webPulseActive()&&!g_simDirections.ready(0));
 command("web calibrate 42 h1 rev 10 1200");tick(200);assert(!webPulseActive()&&g_simDirections.ready(0));
 command("web sim 42 reset");tick(20,false);command("web arm 42");assert(webCalMask()==15);
 command("web program 42 slot 4");assert(g_programSlot==3&&!g_webProgramEditing);
 command("web testplan 42 50 30 1");assert(g_programStorage.readyForAuto(g_program));
 command("web start 42 auto");assert(g_auto.running()&&g_auto.simulation());tick(5000);assert(!g_auto.running()&&g_auto.phase()==AutoRunnerV6::Phase::DONE&&physicalCalls==0);
 command("web program 42 count 2");assert(g_webProgramEditing);command("web start 42 auto");assert(!g_auto.running());
 command("web program 42 home 1000 1000");command("web program 42 travel 1000 1000");
 command("web zone 42 1 1000 1000 970 970 1 0 0 10 10 1");command("web zone 42 2 1050 1050 970 970 1 0 0 10 10 1");
 command("web program 42 order 2 1");command("web program 42 commit");assert(!g_webProgramEditing);
 command("web program 42 name D0A2D0B5D181D182");command("web program 42 save");AutoProgramV6 saved;assert(g_programStorage.load(3,saved)&&saved.order[0]==1);
 command("web settings-save 42");for(unsigned i=0;i<1856;i++)assert(EEPROM.bytes[i]==0x5A);
 assert(EEPROM.bytes[4000]==0); // real direction calibration is never saved by simulation
 command("web start 42 auto");assert(g_auto.running());command("web sim 42 sensor v1 1");tick(20);assert(!g_webLease.armed()&&!g_auto.running()&&g_simPlant.stopped());
 command("web sim 42 sensor v1 0");tick(15,false);command("web arm 42");assert(g_webLease.armed());
 command("web sim 42 estop 1");tick(1);assert(!g_webLease.armed());command("web clear 42");assert(g_simPlant.estopLatched);
 command("web sim 42 estop 0");command("web clear 42");assert(!g_simPlant.estopLatched);
 command("web arm 42");command("web sim 42 startup 1");command("web jog 42 v1 pos 10");assert(!g_webJogMask);command("web sim 42 startup 0");
 command("web jog 42 v1 pos 10");assert(g_webJogMask);command("web sim 42 offline v1 1");tick(1);assert(!g_webLease.armed()&&g_simPlant.stopped());
 command("web sim 42 offline v1 0");tick(12,false);command("web arm 42");assert(g_webLease.armed());tick(85,false);assert(!g_webLease.armed());
 assert(Serial.requests==0&&physicalCalls==0);puts("PASS: actual SIM web parser plus external AutoRunner/plant; jog, two-zone DONE, transfer commit, slot 4, EEPROM isolation, faults/E-STOP/frequency guard/lease; zero physical I/O");
}
