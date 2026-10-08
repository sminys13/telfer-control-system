#pragma once
#include <stdint.h>
#include <string.h>
#include "he200_parameters_v6.h"

// No Arduino I/O, UART, Modbus or EEPROM here. A deliberately simple training
// model, not a prediction of hoist/brake mechanics or HE200 firmware internals.
class SimulationPlantV6 {
public:
  static constexpr uint8_t PARAM_COUNT = sizeof(HE200_WEB_PARAMETERS)/sizeof(HE200_WEB_PARAMETERS[0]);
  uint8_t sensorLostMask=0, offlineMask=0, faultMask=0, stuckMask=0;
  bool estop=false, estopLatched=false;
  uint8_t rate=1;

  void begin(uint32_t now) {
    _last=now;_clock=now;_sample=now;
    sensorLostMask=offlineMask=faultMask=stuckMask=0;estop=estopLatched=false;rate=1;
    memset(_parameter,0,sizeof(_parameter));
    for(uint8_t i=0;i<4;i++){
      _position[i]=1000000;_request[i]=_actual[i]=0;
      put(i,0xF002,2);put(i,0xF003,9);put(i,0xF00A,5000);
      put(i,0xF011,i<2?30:40);put(i,0xF012,i<2?10:0);
      put(i,0xF013,1);put(i,0xF016,2);
      put(i,0xF400,1);put(i,0xF401,2);put(i,0xF402,10);put(i,0xF403,9);
      put(i,0xF009,i==3?1:0);put(i,0xF600,i>=2?2:0);
      put(i,0xF605,i>=2?30:0);put(i,0xF606,i>=2?3:0);
    }
  }
  uint32_t clock() const {return _clock;}
  int32_t raw(uint8_t i) const {return i<4?_position[i]/1000:0;}
  bool setPosition(uint8_t i,int32_t mm) {if(i>=4||mm<0||mm>10000||moving())return false;_position[i]=mm*1000;return true;}
  int8_t forwardSign(uint8_t i) const {return (i<2?-1:1)*(get(i,0xF009)?-1:1);}
  int16_t requested(uint8_t i) const {return i<4?_request[i]:0;}
  uint16_t frequency(uint8_t i) const {return uint32_t(magnitude(_actual[i]))*get(i,0xF00A)/10000UL;}
  uint16_t setFrequency(uint8_t i) const {return uint32_t(magnitude(_request[i]))*get(i,0xF00A)/10000UL;}
  uint16_t outputVoltage(uint8_t i) const {return uint32_t(frequency(i))*230/5000;}
  bool connected(uint8_t i) const {return i<4&&!(offlineMask&(1u<<i));}
  bool healthy() const {return !offlineMask&&!faultMask;}
  bool moving() const {for(uint8_t i=0;i<4;i++)if(_actual[i]||_request[i])return true;return false;}
  bool stopped() const {return !moving();}
  bool sampleDue(uint32_t now) {if(uint32_t(now-_sample)<uint32_t(200/(rate?rate:1)))return false;_sample=now;return true;}
  bool read(uint8_t i,uint16_t reg,uint16_t& value) const {
    if(!connected(i))return false;
    const int8_t index=indexOf(reg);if(index<0)return false;value=_parameter[i][index];return true;
  }
  bool write(uint8_t i,uint16_t reg,uint16_t value) {
    const int8_t index=indexOf(reg);He200ParameterV6 p;
    if(index<0||!connected(i)||moving())return false;
    memcpy_P(&p,&HE200_WEB_PARAMETERS[index],sizeof(p));
    if(!p.writable||value<p.min||value>p.max)return false;
    _parameter[i][index]=value;return true;
  }
  void command(uint8_t i,int16_t rawPercent) {
    if(i>=4)return;
    if(rawPercent>100)rawPercent=100;
    if(rawPercent<-100)rawPercent=-100;
    _request[i]=rawPercent*100;
  }
  void stop(bool immediate=false) {
    for(uint8_t i=0;i<4;i++){_request[i]=0;if(immediate)_actual[i]=0;}
  }
  void emergency(bool active) {estop=active;if(active){estopLatched=true;stop(true);}}
  bool clearLatch(){if(estop)return false;estopLatched=false;return true;}
  void startupScenario(bool blocked) {
    if(moving())return;
    for(uint8_t i=2;i<4;i++){put(i,0xF603,blocked?1000:0);put(i,0xF00E,blocked?1000:0);}
  }
  bool startupScenarioActive()const{return get(2,0xF603)==1000&&get(3,0xF603)==1000&&get(2,0xF00E)==1000&&get(3,0xF00E)==1000;}
  void tick(uint32_t now) {
    uint32_t dt=now-_last;_last=now;if(dt>200)dt=200;
    const uint32_t scaled=dt*(rate>=1&&rate<=5?rate:1);_clock+=scaled;
    if(estop||estopLatched){stop(true);return;}
    for(uint8_t i=0;i<4;i++){
      int16_t target=_request[i];const uint16_t maximum=get(i,0xF00A);
      const uint16_t requestedHz=uint32_t(magnitude(target))*maximum/10000UL;
      if(!connected(i)||(faultMask&(1u<<i))||(target&&requestedHz<get(i,0xF603))||
         (target<0&&get(i,0xF80D)))target=0;
      if(target&&requestedHz<get(i,0xF00E)){
        const uint16_t mode=get(i,0xF80E);
        if(mode)target=0;
        else if(maximum){const int16_t floor=uint32_t(get(i,0xF00E))*10000/maximum;target=target>0?floor:-floor;}
      }
      const bool decel=magnitude(target)<magnitude(_actual[i])||int32_t(target)*_actual[i]<0;
      const uint32_t resolution=get(i,0xF013)==0?1000:get(i,0xF013)==1?100:10;
      const uint32_t duration=uint32_t(get(i,decel?0xF012:0xF011))*resolution;
      int32_t amount=duration?10000UL*scaled/duration:20000;
      if(amount<1&&scaled)amount=1;
      if(target>_actual[i])_actual[i]+=target-_actual[i]>amount?amount:target-_actual[i];
      else if(target<_actual[i])_actual[i]-=_actual[i]-target>amount?amount:_actual[i]-target;
      if(!(stuckMask&(1u<<i))){
        // 100% = 200 mm/s. Fixed-point micrometre coordinates preserve tiny steps.
        _position[i]+=int32_t(_actual[i])*200*int32_t(scaled)/10000*forwardSign(i);
        if(_position[i]<0){_position[i]=0;_actual[i]=_request[i]=0;}
        if(_position[i]>10000000L){_position[i]=10000000L;_actual[i]=_request[i]=0;}
      }
    }
  }
private:
  int32_t _position[4]={};
  int16_t _request[4]={},_actual[4]={};
  uint16_t _parameter[4][PARAM_COUNT]={};
  uint32_t _last=0,_clock=0,_sample=0;
  static int16_t magnitude(int16_t v){return v<0?-v:v;}
  static int8_t indexOf(uint16_t reg){
    He200ParameterV6 p;for(uint8_t j=0;j<PARAM_COUNT;j++){memcpy_P(&p,&HE200_WEB_PARAMETERS[j],sizeof(p));if(p.reg==reg)return j;}return -1;
  }
  uint16_t get(uint8_t i,uint16_t reg) const {const int8_t j=indexOf(reg);return i<4&&j>=0?_parameter[i][j]:0;}
  void put(uint8_t i,uint16_t reg,uint16_t value){const int8_t j=indexOf(reg);if(i<4&&j>=0)_parameter[i][j]=value;}
};
