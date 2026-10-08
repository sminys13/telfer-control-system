#pragma once

#include <stdint.h>

// Step9H does not yet command a physical stop.  It evaluates every accepted
// laser sample and reports when a production controller WOULD have to stop.
// The class deliberately has no Arduino dependency so the state machine can be
// unit-tested on a desktop compiler.

enum class SensorGuardStateV6 : uint8_t {
  NoData = 0,
  Healthy = 1,
  Warning = 2,
  Lost = 3,
  Fault = 4,
  Recovering = 5
};

struct SensorGuardConfigV6 {
  uint16_t warningAgeMs = 450;
  uint16_t shadowStopAgeMs = 800;
  uint16_t faultAgeMs = 1500;
  uint16_t nominalPeriodMs = 200;   // 5 Hz operating profile
  uint16_t diagnosticMaxVelocityMmS = 500;
  uint16_t diagnosticJumpMarginMm = 80;
  uint8_t recoveryFrames = 5;
};

struct SensorGuardStatsV6 {
  uint32_t acceptedFrames = 0;
  uint32_t intervalCount = 0;
  uint32_t intervalSumMs = 0;
  uint32_t minIntervalMs = 0;
  uint32_t maxIntervalMs = 0;

  // Inter-frame interval histogram:
  // 0 <=250; 1 <=400; 2 <=600; 3 <=800; 4 <=1200;
  // 5 <=1500; 6 >1500 ms.
  uint32_t intervalBins[7] = {0, 0, 0, 0, 0, 0, 0};

  uint16_t warningGapCount = 0;
  uint16_t shadowStopGapCount = 0;
  uint16_t faultGapCount = 0;

  uint32_t maxGapMs = 0;
  uint32_t maxGapStartMs = 0;
  uint32_t maxGapEndMs = 0;
  int32_t maxGapStartMm = 0;
  int32_t maxGapEndMm = 0;

  // Diagnostic only. Step9H never rejects a reading on this basis because a
  // swinging basket can create a real fast change. This counter tells us where
  // a future axis-specific plausibility model is needed.
  uint16_t suspiciousJumpCount = 0;
  uint32_t largestJumpMm = 0;
  uint32_t largestJumpDtMs = 0;
  int32_t largestJumpFromMm = 0;
  int32_t largestJumpToMm = 0;
  uint32_t maxObservedSpeedMmS = 0;
};

struct SensorGuardEventV6 {
  bool stateChanged = false;
  SensorGuardStateV6 previousState = SensorGuardStateV6::NoData;
  SensorGuardStateV6 currentState = SensorGuardStateV6::NoData;

  bool shadowStopChanged = false;
  bool shadowStopActive = false;

  bool gapClosed = false;
  uint32_t gapMs = 0;
  int32_t gapStartMm = 0;
  int32_t gapEndMm = 0;
};

class SensorGuardV6 {
public:
  explicit SensorGuardV6(const SensorGuardConfigV6& config = SensorGuardConfigV6());

  void configure(const SensorGuardConfigV6& config);
  void reset(uint32_t nowMs = 0);
  void setEnabled(bool enabled, uint32_t nowMs);
  SensorGuardEventV6 setMotionMarked(bool moving, uint32_t nowMs);

  SensorGuardEventV6 observe(uint32_t sampleMs, int32_t valueMm);
  SensorGuardEventV6 service(uint32_t nowMs);

  SensorGuardStateV6 state() const { return _state; }
  bool enabled() const { return _enabled; }
  bool motionMarked() const { return _motionMarked; }
  bool shadowStopActive() const { return _shadowStopActive; }
  bool hasData() const { return _lastGoodMs != 0; }
  uint32_t lastGoodMs() const { return _lastGoodMs; }
  int32_t lastValueMm() const { return _lastValueMm; }
  uint8_t recoveryStreak() const { return _recoveryStreak; }
  const SensorGuardConfigV6& config() const { return _config; }
  const SensorGuardStatsV6& stats() const { return _stats; }

  uint32_t ageMs(uint32_t nowMs) const;
  uint32_t averageIntervalMs() const;
  uint32_t estimatedBlindTravelMm() const;

  static const char* stateName(SensorGuardStateV6 state);

private:
  SensorGuardStateV6 calculateState(uint32_t nowMs) const;
  SensorGuardEventV6 applyState(uint32_t nowMs, bool gapClosed,
                                uint32_t gapMs, int32_t gapStartMm,
                                int32_t gapEndMm);
  bool calculateShadowStop(SensorGuardStateV6 state) const;
  void noteInterval(uint32_t dtMs, int32_t fromMm, int32_t toMm,
                    uint32_t startMs, uint32_t endMs);
  static void saturatingIncrement(uint16_t& value);
  static uint32_t absoluteDifference(int32_t a, int32_t b);

private:
  SensorGuardConfigV6 _config;
  SensorGuardStatsV6 _stats;
  SensorGuardStateV6 _state = SensorGuardStateV6::NoData;
  uint32_t _lastGoodMs = 0;
  int32_t _lastValueMm = 0;
  bool _enabled = true;
  bool _motionMarked = false;
  bool _shadowStopActive = false;
  bool _recovering = false;
  uint8_t _recoveryStreak = 0;
};
