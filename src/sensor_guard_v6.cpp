#include "sensor_guard_v6.h"

#include <limits.h>

SensorGuardV6::SensorGuardV6(const SensorGuardConfigV6& config)
  : _config(config) {}

void SensorGuardV6::configure(const SensorGuardConfigV6& config) {
  _config = config;
  if (_config.warningAgeMs == 0) _config.warningAgeMs = 1;
  if (_config.shadowStopAgeMs <= _config.warningAgeMs) {
    _config.shadowStopAgeMs = (uint16_t)(_config.warningAgeMs + 1U);
  }
  if (_config.faultAgeMs <= _config.shadowStopAgeMs) {
    _config.faultAgeMs = (uint16_t)(_config.shadowStopAgeMs + 1U);
  }
  if (_config.recoveryFrames == 0) _config.recoveryFrames = 1;
}

void SensorGuardV6::reset(uint32_t nowMs) {
  (void)nowMs;
  _stats = SensorGuardStatsV6{};
  _state = SensorGuardStateV6::NoData;
  _lastGoodMs = 0;
  _lastValueMm = 0;
  _shadowStopActive = false;
  _recovering = false;
  _recoveryStreak = 0;
}

void SensorGuardV6::setEnabled(bool enabled, uint32_t nowMs) {
  _enabled = enabled;
  (void)service(nowMs);
}

SensorGuardEventV6 SensorGuardV6::setMotionMarked(bool moving,
                                                   uint32_t nowMs) {
  _motionMarked = moving;
  return applyState(nowMs, false, 0, 0, 0);
}

SensorGuardEventV6 SensorGuardV6::observe(uint32_t sampleMs,
                                          int32_t valueMm) {
  const bool hadData = (_lastGoodMs != 0);
  const uint32_t previousMs = _lastGoodMs;
  const int32_t previousMm = _lastValueMm;
  const uint32_t dtMs = hadData ? (uint32_t)(sampleMs - previousMs) : 0;

  bool gapClosed = false;
  if (hadData) {
    noteInterval(dtMs, previousMm, valueMm, previousMs, sampleMs);
    gapClosed = dtMs > _config.warningAgeMs;
  }

  _lastGoodMs = sampleMs;
  _lastValueMm = valueMm;
  ++_stats.acceptedFrames;

  if (!hadData) {
    _recovering = false;
    _recoveryStreak = 0;
  } else if (dtMs > _config.shadowStopAgeMs) {
    // A real controller would already have issued STOP. Require several fresh
    // samples before declaring the signal healthy again.
    _recovering = true;
    _recoveryStreak = 1;
  } else if (_recovering) {
    if (dtMs <= _config.warningAgeMs) {
      if (_recoveryStreak < UINT8_MAX) ++_recoveryStreak;
      if (_recoveryStreak >= _config.recoveryFrames) {
        _recovering = false;
        _recoveryStreak = 0;
      }
    } else {
      _recoveryStreak = 1;
    }
  }

  return applyState(sampleMs, gapClosed, dtMs, previousMm, valueMm);
}

SensorGuardEventV6 SensorGuardV6::service(uint32_t nowMs) {
  return applyState(nowMs, false, 0, 0, 0);
}

uint32_t SensorGuardV6::ageMs(uint32_t nowMs) const {
  return _lastGoodMs ? (uint32_t)(nowMs - _lastGoodMs) : UINT32_MAX;
}

uint32_t SensorGuardV6::averageIntervalMs() const {
  return _stats.intervalCount
      ? (_stats.intervalSumMs / _stats.intervalCount)
      : 0;
}

uint32_t SensorGuardV6::estimatedBlindTravelMm() const {
  return (uint32_t)(((uint64_t)_stats.maxGapMs *
                     _config.diagnosticMaxVelocityMmS + 999ULL) / 1000ULL);
}

const char* SensorGuardV6::stateName(SensorGuardStateV6 state) {
  switch (state) {
    case SensorGuardStateV6::NoData: return "NO_DATA";
    case SensorGuardStateV6::Healthy: return "HEALTHY";
    case SensorGuardStateV6::Warning: return "WARNING";
    case SensorGuardStateV6::Lost: return "LOST";
    case SensorGuardStateV6::Fault: return "FAULT";
    case SensorGuardStateV6::Recovering: return "RECOVERING";
    default: return "UNKNOWN";
  }
}

SensorGuardStateV6 SensorGuardV6::calculateState(uint32_t nowMs) const {
  if (!_enabled || _lastGoodMs == 0) return SensorGuardStateV6::NoData;
  const uint32_t age = nowMs - _lastGoodMs;
  if (age > _config.faultAgeMs) return SensorGuardStateV6::Fault;
  if (age > _config.shadowStopAgeMs) return SensorGuardStateV6::Lost;
  if (_recovering) return SensorGuardStateV6::Recovering;
  if (age > _config.warningAgeMs) return SensorGuardStateV6::Warning;
  return SensorGuardStateV6::Healthy;
}

SensorGuardEventV6 SensorGuardV6::applyState(uint32_t nowMs,
                                             bool gapClosed,
                                             uint32_t gapMs,
                                             int32_t gapStartMm,
                                             int32_t gapEndMm) {
  SensorGuardEventV6 event;
  event.previousState = _state;
  event.currentState = calculateState(nowMs);
  event.stateChanged = event.currentState != event.previousState;
  _state = event.currentState;

  const bool nextShadowStop = calculateShadowStop(_state);
  event.shadowStopChanged = nextShadowStop != _shadowStopActive;
  event.shadowStopActive = nextShadowStop;
  _shadowStopActive = nextShadowStop;

  event.gapClosed = gapClosed;
  event.gapMs = gapMs;
  event.gapStartMm = gapStartMm;
  event.gapEndMm = gapEndMm;
  return event;
}

bool SensorGuardV6::calculateShadowStop(SensorGuardStateV6 state) const {
  if (!_enabled || !_motionMarked) return false;
  return state == SensorGuardStateV6::NoData ||
         state == SensorGuardStateV6::Lost ||
         state == SensorGuardStateV6::Fault ||
         state == SensorGuardStateV6::Recovering;
}

void SensorGuardV6::noteInterval(uint32_t dtMs, int32_t fromMm,
                                 int32_t toMm, uint32_t startMs,
                                 uint32_t endMs) {
  ++_stats.intervalCount;
  _stats.intervalSumMs += dtMs;
  if (_stats.minIntervalMs == 0 || dtMs < _stats.minIntervalMs) {
    _stats.minIntervalMs = dtMs;
  }
  if (dtMs > _stats.maxIntervalMs) _stats.maxIntervalMs = dtMs;

  uint8_t bin = 6;
  if (dtMs <= 250U) bin = 0;
  else if (dtMs <= 400U) bin = 1;
  else if (dtMs <= 600U) bin = 2;
  else if (dtMs <= 800U) bin = 3;
  else if (dtMs <= 1200U) bin = 4;
  else if (dtMs <= 1500U) bin = 5;
  ++_stats.intervalBins[bin];

  if (dtMs > _config.warningAgeMs) saturatingIncrement(_stats.warningGapCount);
  if (dtMs > _config.shadowStopAgeMs) saturatingIncrement(_stats.shadowStopGapCount);
  if (dtMs > _config.faultAgeMs) saturatingIncrement(_stats.faultGapCount);

  if (dtMs > _stats.maxGapMs) {
    _stats.maxGapMs = dtMs;
    _stats.maxGapStartMs = startMs;
    _stats.maxGapEndMs = endMs;
    _stats.maxGapStartMm = fromMm;
    _stats.maxGapEndMm = toMm;
  }

  const uint32_t jumpMm = absoluteDifference(fromMm, toMm);
  if (dtMs > 0) {
    const uint32_t speed = (uint32_t)(((uint64_t)jumpMm * 1000ULL) / dtMs);
    if (speed > _stats.maxObservedSpeedMmS) _stats.maxObservedSpeedMmS = speed;

    const uint32_t allowed =
        (uint32_t)(((uint64_t)_config.diagnosticMaxVelocityMmS * dtMs) / 1000ULL) +
        _config.diagnosticJumpMarginMm;
    if (jumpMm > allowed) {
      saturatingIncrement(_stats.suspiciousJumpCount);
      if (jumpMm > _stats.largestJumpMm) {
        _stats.largestJumpMm = jumpMm;
        _stats.largestJumpDtMs = dtMs;
        _stats.largestJumpFromMm = fromMm;
        _stats.largestJumpToMm = toMm;
      }
    }
  }
}

void SensorGuardV6::saturatingIncrement(uint16_t& value) {
  if (value != UINT16_MAX) ++value;
}

uint32_t SensorGuardV6::absoluteDifference(int32_t a, int32_t b) {
  const int64_t diff = (int64_t)a - (int64_t)b;
  return (uint32_t)(diff < 0 ? -diff : diff);
}
