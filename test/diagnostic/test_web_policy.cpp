#include "Arduino.h"
#include "web_control_policy_v6.h"
#include "he200_parameters_v6.h"
#include "direction_calibration_v6.h"
#include <assert.h>
uint32_t testClock=0;
HardwareSerial Serial;
int main(){
 WebLeaseV6 lease;
 assert(!lease.armed()&&!lease.expired(9999));lease.arm(100);
 assert(!lease.expired(1600)&&lease.expired(1601));lease.ping(1000);assert(!lease.expired(2000));lease.disarm();assert(!lease.armed());
 lease.arm(0xfffffff0UL);assert(!lease.expired(1000));assert(lease.expired(2000));
 uint32_t v=0;assert(webUnsigned("4294967295",0xffffffffUL,v)&&v==0xffffffffUL);
 const char* invalid[]={"","-1","1x","1.0","4294967296","999999999999999999999999999"," 1"};
 for(auto s:invalid)assert(!webUnsigned(s,0xffffffffUL,v));
 assert(webUnsigned("36000",36000,v));assert(!webUnsigned("36001",36000,v));
 He200ParameterV6 p;assert(he200WebParameter("P6.03",p)&&p.reg==0xF603&&p.writable&&p.max==1000);
 assert(he200WebParameter("P4.02",p)&&!p.writable);
 assert(he200WebParameter("P8.18",p)&&!p.writable);
 assert(!he200WebParameter("0x2000",p)&&!he200WebParameter("undefined",p));
 DirectionCalibrationV6 direction;direction.restore(0,-1);assert(direction.ready(0)&&direction.physicalSign(0,true)==-1);
 direction.restore(1,2);assert(!direction.ready(1));direction.reset();assert(!direction.ready(0));
 assert(webCoordinatePercent(10,-1,false,40)==-10);
 assert(webCoordinatePercent(10,-1,true,40)==10);
 assert(webCoordinatePercent(-80,1,false,40)==-40);
 assert(webCoordinatePercent(10,0,false,40)==0);
 puts("PASS: deadman lease including millis wrap; exact bounded integer parsing; named parameter allowlist; measured direction restoration");
}
