#include <cassert>
#include <cstdint>
#include <iostream>

#include "sensor_guard_v6.h"

static SensorGuardConfigV6 config() {
  SensorGuardConfigV6 cfg;
  cfg.warningAgeMs = 450;
  cfg.shadowStopAgeMs = 800;
  cfg.faultAgeMs = 1500;
  cfg.nominalPeriodMs = 200;
  cfg.diagnosticMaxVelocityMmS = 250;
  cfg.diagnosticJumpMarginMm = 80;
  cfg.recoveryFrames = 5;
  return cfg;
}

int main() {
  SensorGuardV6 guard(config());
  guard.reset(0);

  auto event = guard.observe(100, 1000);
  assert(event.stateChanged);
  assert(guard.state() == SensorGuardStateV6::Healthy);
  assert(!guard.shadowStopActive());

  guard.setMotionMarked(true, 100);
  event = guard.service(551); // age 451 ms
  assert(guard.state() == SensorGuardStateV6::Warning);
  assert(!guard.shadowStopActive());

  event = guard.service(901); // age 801 ms
  assert(guard.state() == SensorGuardStateV6::Lost);
  assert(guard.shadowStopActive());
  assert(event.shadowStopChanged);

  event = guard.observe(1000, 1100); // 900 ms inter-frame gap
  assert(event.gapClosed);
  assert(event.gapMs == 900);
  assert(guard.state() == SensorGuardStateV6::Recovering);
  assert(guard.shadowStopActive());
  assert(guard.stats().shadowStopGapCount == 1);

  // Five consecutive fresh frames clear recovery, including the first frame
  // that closed the gap.
  guard.observe(1200, 1120);
  guard.observe(1400, 1140);
  guard.observe(1600, 1160);
  event = guard.observe(1800, 1180);
  assert(guard.state() == SensorGuardStateV6::Healthy);
  assert(!guard.shadowStopActive());

  event = guard.service(3301); // >1500 ms since last frame
  assert(guard.state() == SensorGuardStateV6::Fault);
  assert(guard.shadowStopActive());

  event = guard.observe(3500, 1200);
  assert(event.gapClosed);
  assert(guard.state() == SensorGuardStateV6::Recovering);
  assert(guard.stats().faultGapCount == 1);
  assert(guard.stats().maxGapMs == 1700);
  assert(guard.estimatedBlindTravelMm() == 425);

  // Diagnostic jump is counted but the sample is still accepted.
  guard.observe(3700, 2000);
  assert(guard.stats().suspiciousJumpCount >= 1);
  assert(guard.lastValueMm() == 2000);

  guard.setMotionMarked(false, 3700);
  assert(!guard.shadowStopActive());

  std::cout << "sensor_guard_v6: all tests passed\n";
  return 0;
}
