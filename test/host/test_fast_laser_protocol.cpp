#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>

#define private public
#include "fast_laser_sensor.h"
#include "config_v6_bringup.h"
#undef private

static uint8_t checksum(const uint8_t* data, size_t n) {
  uint8_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum = static_cast<uint8_t>(sum + data[i]);
  return static_cast<uint8_t>(~sum + 1);
}

int main() {
  Sc16Is752 bus(10, 14745600UL);
  FastLaserSensor sensor(bus, Sc16Is752::Channel::A);

  uint8_t measurement[11] = {0x80, 0x06, 0x83, '0','0','8','.','8','0','6', 0};
  measurement[10] = checksum(measurement, 10);
  std::memcpy(sensor._rx, measurement, sizeof(measurement));
  sensor._rxLen = sizeof(measurement);
  int32_t mm = 0;
  assert(sensor.tryConsumeFrame(1000, mm) == 1);
  assert(mm == 8806);
  assert(sensor._diag.streamFrames == 1);

  uint8_t measurement01[12] = {0x80, 0x06, 0x83, '0','0','8','.','8','0','6','5', 0};
  measurement01[11] = checksum(measurement01, 11);
  std::memcpy(sensor._rx, measurement01, sizeof(measurement01));
  sensor._rxLen = sizeof(measurement01);
  assert(sensor.tryConsumeFrame(1050, mm) == 1);
  assert(mm == 8807); // 8.8065 m is rounded to the nearest millimetre

  uint8_t err16[11] = {0x80, 0x06, 0x83, 'E','R','R','-','-','1','6', 0};
  err16[10] = checksum(err16, 10);
  std::memcpy(sensor._rx, err16, sizeof(err16));
  sensor._rxLen = sizeof(err16);
  assert(sensor.tryConsumeFrame(1100, mm) == 0);
  assert(sensor._diag.sensorErrorFrames == 1);
  assert(sensor._diag.lastSensorError == 16);
  assert(sensor._diag.errorCode16Frames == 1);
  assert(sensor._diag.maxErrorStreak == 1);

  uint8_t err15[11] = {0x80, 0x06, 0x82, 'E','R','R','-','-','1','5', 0};
  err15[10] = checksum(err15, 10);
  sensor._mode = FastLaserAcquisitionMode::SingleShot;
  sensor._state = FastLaserSensor::State::Running;
  sensor._singlePending = true;
  sensor._diag.singlePending = true;
  std::memcpy(sensor._rx, err15, sizeof(err15));
  sensor._rxLen = sizeof(err15);
  assert(sensor.tryConsumeFrame(1150, mm) == 0);
  assert(sensor._diag.errorCode15Frames == 1);
  assert(sensor._diag.singleResponses == 1);
  assert(!sensor._singlePending);

  const uint8_t ackFreq[] = {0xFA, 0x04, 0x8A, 0x78};
  std::memcpy(sensor._rx, ackFreq, sizeof(ackFreq));
  sensor._rxLen = sizeof(ackFreq);
  assert(sensor.tryConsumeFrame(1200, mm) == 0);
  assert((sensor._diag.ackMask & FAST_LASER_ACK_FREQUENCY) != 0);

  uint8_t bad[11];
  std::memcpy(bad, measurement, sizeof(bad));
  bad[10] ^= 0x01;
  std::memcpy(sensor._rx, bad, sizeof(bad));
  sensor._rxLen = sizeof(bad);
  assert(sensor.tryConsumeFrame(1300, mm) == -1); // waits for possible 12-byte frame
  sensor._rx[sensor._rxLen++] = 0x00;
  assert(sensor.tryConsumeFrame(1301, mm) == 0);
  assert(sensor._diag.checksumErrors == 1);

  sensor._diag.configuring = false;
  sensor._state = FastLaserSensor::State::Running;
  sensor._mode = FastLaserAcquisitionMode::SingleShot;
  sensor._singlePending = false;
  assert(sensor.requestSingleMeasurement(2000));
  assert(sensor._diag.singleRequests == 1);
  assert(sensor.singleRequestPending());
  std::cout << "fast_laser_protocol_tests_step9h: OK\n";
  return 0;
}
