#pragma once
#include <EEPROM.h>
#include <string.h>
#include <stddef.h>
#include "config_v6_bringup.h"
#include "utils.h"

class ProgramLabelsV6 {
public:
  static constexpr uint8_t BYTES=40;
  void begin(){
    Record r;EEPROM.get(address(),r);
    if(r.magic!=MAGIC||r.crc!=crc16_modbus(reinterpret_cast<const uint8_t*>(&r),offsetof(Record,crc)))memset(_labels,0,sizeof(_labels));
    else{memcpy(_labels,r.label,sizeof(_labels));for(uint8_t i=0;i<4;i++)_labels[i][BYTES-1]=0;}
  }
  const char* name(uint8_t i)const{return i<4?_labels[i]:"";}
  bool setHex(uint8_t slot,const char* hex){
    if(slot>=4||!hex)return false;
    const size_t n=strlen(hex);if(n%2||n>2*(BYTES-1))return false;
    char parsed[BYTES]={};for(size_t i=0;i<n;i+=2){const int8_t a=nibble(hex[i]),b=nibble(hex[i+1]);if(a<0||b<0)return false;const uint8_t value=(a<<4)|b;if(value<32||value==127)return false;parsed[i/2]=value;}
    memcpy(_labels[slot],parsed,BYTES);return true;
  }
  void save(){Record r;memset(&r,0,sizeof(r));r.magic=MAGIC;memcpy(r.label,_labels,sizeof(_labels));r.crc=crc16_modbus(reinterpret_cast<const uint8_t*>(&r),offsetof(Record,crc));EEPROM.put(address(),r);}
private:
  struct Record{uint32_t magic;char label[4][BYTES];uint16_t crc;};
  static constexpr uint32_t MAGIC=0x574C4231UL;
  char _labels[4][BYTES]={};
  static int address(){return DESKTOP_SIMULATION_ENABLED?3800:3600;}
  static int8_t nibble(char c){return c>='0'&&c<='9'?c-'0':c>='a'&&c<='f'?c-'a'+10:c>='A'&&c<='F'?c-'A'+10:-1;}
  static_assert(3800+sizeof(Record)<4000,"labels must fit before real direction calibration");
};
