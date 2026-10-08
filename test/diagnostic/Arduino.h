#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define SERIAL_8N1 6
#define HEX 16
class __FlashStringHelper {};
#define F(s) reinterpret_cast<const __FlashStringHelper*>(s)
extern uint32_t testClock;
inline uint32_t millis() { return ++testClock; }
inline uint32_t micros() { return testClock*1000; }
inline void delayMicroseconds(unsigned) {}
inline void pinMode(uint8_t,uint8_t) {}
inline void digitalWrite(uint8_t,uint8_t) {}
class HardwareSerial {
public:
  unsigned requests=0;
  uint8_t response[64]={}, responseSize=0, pos=0;
  void (*reply)(HardwareSerial&,const uint8_t*,size_t)=nullptr;
  char log[250000]={}; size_t logSize=0;
  void begin(uint32_t,uint16_t) {} void end() {} void flush() {}
  int available() { return responseSize-pos; }
  int read() { return pos<responseSize ? response[pos++] : -1; }
  size_t write(const uint8_t* data,size_t size) {
    ++requests; pos=0;responseSize=0;if(reply)reply(*this,data,size);return size;
  }
  void print(const char* s) { size_t n=strlen(s);if(logSize+n<sizeof(log)){memcpy(log+logSize,s,n+1);logSize+=n;} }
  void print(char* s) {print(static_cast<const char*>(s));}
  void print(const __FlashStringHelper* s) {print(reinterpret_cast<const char*>(s));}
  void print(char v) {char s[]={v,0};print(s);}
  template<class T> void print(T v,int base=10) {char s[40]; if(base==16)snprintf(s,sizeof(s),"%llX",(unsigned long long)v);else snprintf(s,sizeof(s),"%lld",(long long)v);print(s);}
  void println() {print("\n");}
  template<class T> void println(T v) {print(v);println();}
};
extern HardwareSerial Serial;
