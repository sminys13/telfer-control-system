#pragma once
#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>
struct He200ParameterV6 { char key[8];uint16_t reg,min,max;bool writable; };
// Human-named allowlist. DI functions/polarity, addresses, protection resets and
// arbitrary registers are read-only or absent. Units are explicit in the web UI.
const He200ParameterV6 HE200_WEB_PARAMETERS[] PROGMEM = {
 {"P0.02",0xF002,2,2,true},{"P0.03",0xF003,9,9,true},
 {"P0.09",0xF009,0,1,true},{"P0.10",0xF00A,5000,5000,false},
 {"P0.14",0xF00E,0,5000,true},{"P0.17",0xF011,0,65000,true},
 {"P0.18",0xF012,0,65000,true},{"P0.19",0xF013,0,2,false},
 {"P0.22",0xF016,1,2,false},{"P0.27",0xF01B,0,0,true},
 {"P4.00",0xF400,0,49,false},{"P4.01",0xF401,0,49,false},
 {"P4.02",0xF402,0,49,false},{"P4.03",0xF403,0,49,false},
 {"P4.11",0xF40B,0,3,false},{"P4.38",0xF426,0,65535,false},
 {"P6.00",0xF600,0,2,true},{"P6.03",0xF603,0,1000,true},
 {"P6.04",0xF604,0,1000,true},{"P6.05",0xF605,0,100,true},
 {"P6.06",0xF606,0,1000,true},{"P6.10",0xF60A,0,1,true},
 {"P8.12",0xF80C,0,30000,true},{"P8.13",0xF80D,0,1,true},
 {"P8.14",0xF80E,0,2,true},{"P8.18",0xF812,0,1,false}
};
inline bool he200WebParameter(const char* key,He200ParameterV6& out) {
 for(uint8_t i=0;i<sizeof(HE200_WEB_PARAMETERS)/sizeof(HE200_WEB_PARAMETERS[0]);i++) {
  memcpy_P(&out,&HE200_WEB_PARAMETERS[i],sizeof(out));if(!strcmp(key,out.key))return true;
 }
 return false;
}
