#include "Arduino.h"
#include "EEPROM.h"
#include "auto_runner_v6.h"
#include <assert.h>
uint32_t testClock=0;
HardwareSerial Serial;
TestEEPROMV6 EEPROM;
unsigned physicalCalls=0;
ModbusMasterRTU::ModbusMasterRTU(){}
bool MotorControlV6::requestAutoTargets(SystemModeV6,int16_t,int16_t,int16_t,int16_t){physicalCalls++;return true;}
void MotorControlV6::autoStop(const __FlashStringHelper*){physicalCalls++;}
int main(){
 MotorControlV6 motor;SettingsV6 settings{};
 for(auto& profile:settings.drive)profile={10,40,10,0,100,8};
 ProgramStorageV6 storage;AutoProgramV6 program;storage.demo(program);assert(storage.readyForAuto(program));
 storage.saveActiveSlot(3);assert(storage.loadActiveSlot()==3);
 assert(storage.save(0,program));AutoProgramV6 loaded;assert(storage.load(0,loaded)&&storage.readyForAuto(loaded));
 AutoSensorsV6 sensors;sensors.usableMask=15;for(auto& mm:sensors.mm)mm=1000;
 AutoRunnerV6 runner;runner.begin(motor,settings);testClock=1000;assert(runner.start(program,sensors,true,testClock));
 for(unsigned i=0;i<10000&&runner.running();i++){
  testClock+=100;runner.service(testClock,sensors,false,0);
  if(i==10){runner.pause();auto phase=runner.phase();testClock+=1000;runner.service(testClock,sensors,false,0);assert(runner.paused()&&runner.phase()==phase);runner.resume(testClock);}
  if(runner.waitOperator())runner.operatorNext();
 }
 if(runner.phase()!=AutoRunnerV6::Phase::DONE)printf("Simulation phase=%u error=%u positions=%d/%d/%d/%d\n",(unsigned)runner.phase(),runner.error(),(int)runner.simPosition(SENSOR_X1),(int)runner.simPosition(SENSOR_X2),(int)runner.simPosition(SENSOR_Z1),(int)runner.simPosition(SENSOR_Z2));
 assert(!runner.running()&&runner.phase()==AutoRunnerV6::Phase::DONE&&runner.error()==0);
 assert(physicalCalls==0);
 assert(runner.startHome(program,sensors,true,testClock));
 for(unsigned i=0;i<10000&&runner.running();i++){testClock+=100;runner.service(testClock,sensors,false,0);}
 assert(runner.phase()==AutoRunnerV6::Phase::DONE&&physicalCalls==0);
 program.dryingEnabled=1;program.dryValid=1;program.dryingTimeS=1;program.stagingZone=0;
 program.validMask|=AUTO_PROGRAM_VALID_DRY_X|AUTO_PROGRAM_VALID_DRY_Z;
 program.dryX[0]=program.dryX[1]=1500;program.dryZ[0]=program.dryZ[1]=300;
 assert(storage.readyForAuto(program));assert(runner.start(program,sensors,true,testClock));unsigned operatorWaits=0;
 for(unsigned i=0;i<10000&&runner.running();i++){testClock+=100;runner.service(testClock,sensors,false,0);if(runner.waitOperator()){operatorWaits++;runner.operatorNext();}}
 assert(runner.phase()==AutoRunnerV6::Phase::DONE&&operatorWaits>=4&&physicalCalls==0);
 assert(runner.start(program,sensors,true,testClock));runner.service(testClock,sensors,true,0);assert(runner.error()==AUTO_ERR_ESTOP&&physicalCalls==0);
 program.validMask=0;assert(!runner.start(program,sensors,true,testClock));
 puts("PASS: actual AutoRunner two-zone cycle, dwell/tilt/HOME, drying/operator steps, pause/resume, E-STOP, invalid program; zero physical calls; EEPROM save/load");
}
