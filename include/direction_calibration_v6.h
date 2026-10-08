#pragma once
#include <stdint.h>

// Raw laser distance, independent of EEPROM offsets/inversion and UP/DOWN labels.
// Two opposite physical commands must produce two opposite measured signs.
class DirectionCalibrationV6 {
public:
  struct Window {
    int32_t mm[3] = {};
    uint32_t stamp[3] = {};
    uint8_t n = 0;
    void add(int32_t value, uint32_t ms) {
      if (value<0 || value>10000) { n=0; return; }
      if (!ms || (n && stamp[n-1]==ms)) return;
      if (n==3) { mm[0]=mm[1]; mm[1]=mm[2]; stamp[0]=stamp[1]; stamp[1]=stamp[2]; n=2; }
      mm[n]=value; stamp[n]=ms; ++n;
    }
    bool stable(uint32_t now, uint32_t after=0) const {
      if (n!=3 || (uint32_t)(now-stamp[2])>450 || (uint32_t)(now-stamp[0])>1200) return false;
      if (after && (int32_t)(stamp[0]-after)<=0) return false;
      int32_t low=mm[0], high=mm[0];
      for (uint8_t i=1;i<3;++i) { if (mm[i]<low) low=mm[i]; if (mm[i]>high) high=mm[i]; }
      return high-low<=5;
    }
    int32_t mean() const { return (mm[0]+mm[1]+mm[2])/3; }
  };
  Window samples[4];
  void reset() { _armed=false; for (uint8_t i=0;i<4;++i) { _fwd[i]=0; _rev[i]=0; } }
  void revoke(uint8_t drive) { if (drive<4) _fwd[drive]=_rev[drive]=0; }
  bool ready(uint8_t drive) const { return drive<4 && _fwd[drive] && _rev[drive]==-_fwd[drive]; }
  void restore(uint8_t drive, int8_t forwardSign) {
    if(drive<4 && (forwardSign==1 || forwardSign==-1)) { _fwd[drive]=forwardSign; _rev[drive]=-forwardSign; }
  }
  int8_t physicalSign(uint8_t drive, bool positive) const {
    return ready(drive) ? (positive ? _fwd[drive] : -_fwd[drive]) : 0;
  }
  bool begin(uint8_t drive, uint32_t now) {
    _armed=false;
    if (drive>=4) return false;
    for (uint8_t i=0;i<4;++i) if (!samples[i].stable(now)) return false;
    for (uint8_t i=0;i<4;++i) _before[i]=samples[i].mean();
    _armed=true; _drive=drive; _started=now;
    return true;
  }
  // No calibration on stale samples, unconfirmed electrical RUN/STOP, sensor noise,
  // or displacement on another axis (cross-wiring/coupled load).
  bool finish(uint8_t drive, bool forward, uint32_t now, uint32_t stopMs,
              bool ranAndStopped, int32_t& delta) {
    delta=0;
    if (drive>=4) return false;
    const bool armed=_armed && drive==_drive && (int32_t)(stopMs-_started)>0;
    _armed=false;
    if (!armed) { revoke(drive); return false; }
    if (!ranAndStopped) { revoke(drive); return false; }
    for (uint8_t i=0;i<4;++i) {
      if (!samples[i].stable(now,stopMs)) { revoke(drive); return false; }
      const int32_t diff=samples[i].mean()-_before[i];
      if (i==drive) delta=diff;
      else if (diff>5 || diff<-5) { revoke(drive); return false; }
    }
    if (delta>-10 && delta<10) { revoke(drive); return false; }
    const int8_t sign=delta>0 ? 1 : -1;
    int8_t& slot=forward ? _fwd[drive] : _rev[drive];
    if (slot && slot!=sign) { revoke(drive); return false; }
    slot=sign;
    if (_fwd[drive] && _rev[drive] && _fwd[drive]!=-_rev[drive]) { revoke(drive); return false; }
    return true;
  }
private:
  bool _armed=false;
  uint8_t _drive=0;
  uint32_t _started=0;
  int8_t _fwd[4] = {}, _rev[4] = {};
  int32_t _before[4] = {};
};
