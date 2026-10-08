#pragma once
#include <stdint.h>
#if defined(__AVR__)
#include <avr/io.h>
extern char __heap_start;
extern char* __brkval;
#endif
class RamMonitorV6 {
public:
 void begin(){
#if defined(__AVR__)
  _start=reinterpret_cast<uintptr_t>(__brkval?__brkval:&__heap_start);
  _end=uintptr_t(SP)-96;
  if(_end>_start)for(uintptr_t p=_start;p<_end;p++)*reinterpret_cast<volatile uint8_t*>(p)=0xA5;
#endif
 }
 uint16_t freeNow()const{
#if defined(__AVR__)
  return uintptr_t(SP)-reinterpret_cast<uintptr_t>(__brkval?__brkval:&__heap_start);
#else
  return 0;
#endif
 }
 uint16_t minimum()const{
#if defined(__AVR__)
  if(!_start||_end<=_start)return 0;
  uintptr_t p=_start;while(p<_end&&*reinterpret_cast<volatile uint8_t*>(p)==0xA5)++p;return p-_start;
#else
  return 0;
#endif
 }
private:
#if defined(__AVR__)
 uintptr_t _start=0,_end=0;
#endif
};
