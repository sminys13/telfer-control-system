#pragma once
#include <stdint.h>
#include <string.h>
class TestEEPROMV6 {
 public:
 uint8_t bytes[4096]={};
 unsigned writes=0;
 uint16_t length() const {return sizeof(bytes);}
 template<class T> void get(int addr,T& value)const {memcpy(&value,bytes+addr,sizeof(value));}
 template<class T> void put(int addr,const T& value){memcpy(bytes+addr,&value,sizeof(value));writes++;}
 void update(int addr,uint8_t value){bytes[addr]=value;writes++;}
};
extern TestEEPROMV6 EEPROM;
