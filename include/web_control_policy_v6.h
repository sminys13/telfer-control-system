#pragma once
#include <stdint.h>
#include <stddef.h>

class WebLeaseV6 {
public:
  void arm(uint32_t now) { _armed=true;_last=now; }
  void ping(uint32_t now) { if(_armed)_last=now; }
  void disarm() { _armed=false; }
  bool armed() const { return _armed; }
  bool expired(uint32_t now) const { return _armed && (uint32_t)(now-_last)>1500; }
private:
  bool _armed=false;
  uint32_t _last=0;
};

inline bool webUnsigned(const char* text,uint32_t maximum,uint32_t& result) {
  result=0;if(!text || !*text)return false;
  for(const char* p=text;*p;p++) {
    if(*p<'0'||*p>'9')return false;
    const uint8_t digit=*p-'0';
    if(result>maximum/10 || (result==maximum/10 && digit>maximum%10))return false;
    result=result*10+digit;
  }
  return true;
}
inline int16_t webCoordinatePercent(int16_t logical,int8_t forwardSensorSign,bool legacyInvert,uint8_t cap) {
  if(forwardSensorSign!=1&&forwardSensorSign!=-1)return 0;
  if(logical>cap)logical=cap;
  if(logical<-(int16_t)cap)logical=-(int16_t)cap;
  const int16_t raw=logical*forwardSensorSign;
  return legacyInvert?-raw:raw;
}
