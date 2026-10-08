#include "Arduino.h"
#include "modbus.h"
#include "he200_audit_v6.h"
#include "direction_calibration_v6.h"
#include <assert.h>
#include <stdlib.h>
#undef assert
#define assert(x) do { if (!(x)) { printf("FAIL line %d: %s\n",__LINE__,#x); fflush(stdout); exit(1); } } while(0)
uint32_t testClock=0;
HardwareSerial Serial;
uint16_t crc16_modbus(const uint8_t* p,size_t n){uint16_t c=65535;while(n--){c^=*p++;for(int i=0;i<8;i++)c=(c&1)?(c>>1)^0xA001:c>>1;}return c;}
int responseMode=0;
int truncateResponse=-1;
unsigned fc03=0,fcOther=0;
unsigned addressReads[248]={};
void reply(HardwareSerial& s,const uint8_t* req,size_t n){
  assert(n==8);assert(crc16_modbus(req,6)==(req[6]|req[7]<<8));
  if(req[1]==3){++fc03;++addressReads[req[0]];}else ++fcOther;
  const uint16_t reg=(req[2]<<8)|req[3];
  if(responseMode==5)return;
  bool exception=responseMode==1||(responseMode==6&&req[0]==3&&reg==0xF427);
  if(exception){s.response[0]=req[0];s.response[1]=0x83;s.response[2]=2;s.responseSize=5;}
  else {
    const uint8_t words=req[5];assert(words>=1&&words<=8);
    s.response[0]=req[0]+(responseMode==3?1:0);s.response[1]=responseMode==4?4:req[1];s.response[2]=2*words;
    for(uint8_t i=0;i<words;i++){s.response[3+2*i]=0;s.response[4+2*i]=42+i;}
    s.responseSize=5+2*words;
  }
  uint16_t c=crc16_modbus(s.response,s.responseSize-2);
  s.response[s.responseSize-2]=c&255;s.response[s.responseSize-1]=c>>8;
  if(responseMode==2)s.response[s.responseSize-1]^=1;
  if(truncateResponse>=0&&truncateResponse<s.responseSize)s.responseSize=(uint8_t)truncateResponse;
}
void window(DirectionCalibrationV6& c,int32_t target,uint32_t end,int32_t other=1000){
  for(uint8_t i=0;i<4;i++)for(uint8_t j=0;j<3;j++)c.samples[i].add(i?other:target,end-400+200*j);
}
int main(){
  HardwareSerial bus;bus.reply=reply;
  ModbusMasterRTU mb;mb.begin(bus,6,9600,30,SERIAL_8N1,1,true,false,false,true);
  uint16_t value=999;
  auto r=mb.readHoldingRegisters(1,0xF002,1,&value);assert(r.ok&&value==42&&r.exception==0);
  responseMode=1;unsigned before=bus.requests;r=mb.readHoldingRegisters(1,0xF002,1,&value);
  assert(!r.ok&&r.error==MODBUS_ERROR_EXCEPTION&&r.exception==2&&bus.requests==before+1);
  responseMode=2;r=mb.readHoldingRegisters(1,0xF002,1,&value);assert(!r.ok&&r.error==MODBUS_ERROR_CRC);
  responseMode=3;r=mb.readHoldingRegisters(1,0xF002,1,&value);assert(!r.ok&&r.error==MODBUS_ERROR_BAD_RESPONSE);
  responseMode=4;r=mb.readHoldingRegisters(1,0xF002,1,&value);assert(!r.ok&&r.error==MODBUS_ERROR_BAD_RESPONSE);
  responseMode=5;r=mb.readHoldingRegisters(1,0xF002,1,&value);assert(!r.ok&&r.error==MODBUS_ERROR_TIMEOUT);
  before=bus.requests;
  const uint16_t regs[]={0x1000,0x2000,0x2001,0xF002};
  for(auto reg:regs)for(uint16_t v=1;v<=7;v++){r=mb.writeSingleRegister(1,reg,v);assert(!r.ok&&r.error==MODBUS_ERROR_WRITE_LOCKED);}
  assert(bus.requests==before);
  puts("PASS: FC03 validation, CRC/address/function/timeout, exact exception; all physical writes blocked");
  He200AuditV6 audit;uint8_t ids[]={1,2,3,4},bad[]={1,2,3,3};
  unsigned prior[4];for(unsigned i=0;i<4;i++)prior[i]=addressReads[i+1];
  assert(!audit.start(mb,bad));responseMode=6;before=bus.requests;assert(audit.start(mb,ids));assert(!audit.start(mb,ids));
  for(unsigned i=0;audit.active()&&i<1000;i++){testClock+=40;audit.tick(testClock);}
  assert(!audit.active()&&bus.requests-before==372&&fcOther==0);
  for(unsigned i=0;i<4;i++)assert(addressReads[i+1]-prior[i]==93);
  assert(strstr(Serial.log,"state=DONE")&&strstr(Serial.log,"errors=1 motionPermit=0"));
  assert(strstr(Serial.log,"key=P4.39 reg=0xF427 ok=0 error=3 exception=2"));
  assert(audit.start(mb,ids));testClock+=40;audit.tick(testClock);audit.cancel();before=bus.requests;testClock+=40;audit.tick(testClock);assert(bus.requests==before);
  mb.reconfigure(9600,30,SERIAL_8N1,0,false,true,false,true);assert(!audit.start(mb,ids));
  puts("PASS: 372 reads across all four drives; unsupported register remains unknown; cancel; no writes; dry-run cannot pass");
  mb.reconfigure(9600,80,SERIAL_8N1,0,true,false,false,true);responseMode=0;
  for(uint16_t count=1;count<=8;count++)for(uint8_t function=3;function<=4;function++){
    uint16_t values[8]={};
    r=function==3?mb.readHoldingRegisters(1,0x7000,count,values):mb.readInputRegisters(1,0x7000,count,values);
    assert(r.ok);for(uint16_t i=0;i<count;i++)assert(values[i]==42+i);
    for(truncateResponse=0;truncateResponse<5+count*2;truncateResponse++){
      for(uint16_t i=0;i<8;i++)values[i]=0xBEEF;
      r=function==3?mb.readHoldingRegisters(1,0x7000,count,values):mb.readInputRegisters(1,0x7000,count,values);
      assert(!r.ok);for(uint16_t i=0;i<8;i++)assert(values[i]==0xBEEF);
    }
    truncateResponse=-1;
  }
  responseMode=5;mb.reconfigure(9600,5,SERIAL_8N1,255,true,false,false,true);before=bus.requests;
  r=mb.readHoldingRegisters(1,0x7000,1,&value);
  assert(!r.ok&&bus.requests-before==256);
  puts("PASS: FC03/FC04 counts 1..8; 224 truncated frames rejected without modifying values; retries=255 terminates after 256 attempts");
  DirectionCalibrationV6 cal;int32_t delta;
  window(cal,1000,1000);assert(cal.begin(0,1010));window(cal,980,2000);
  assert(cal.finish(0,true,2010,1500,true,delta)&&delta==-20&&!cal.ready(0));
  assert(cal.begin(0,2020));window(cal,1000,3000);
  assert(cal.finish(0,false,3010,2500,true,delta)&&cal.ready(0)&&cal.physicalSign(0,true)==-1);
  assert(!cal.begin(0,6000));assert(!cal.finish(0,false,6000,5000,true,delta)&&!cal.ready(0));
  window(cal,1000,7000);assert(cal.begin(0,7010));window(cal,1004,8000);
  assert(!cal.finish(0,true,8010,7500,true,delta));
  assert(cal.begin(0,8020));window(cal,1020,9000,1020);assert(!cal.finish(0,true,9010,8500,true,delta));
  window(cal,1000,10000);assert(cal.begin(0,10010));window(cal,980,11000);assert(!cal.finish(0,true,11010,10500,false,delta));
  cal.samples[0].add(-1,11000);assert(!cal.begin(0,11020));
  puts("PASS: FWD decreases => coordinate plus is REV; both directions required; stale/noise/cross-axis/no-RUN/invalid rejected");
}
