\
/**
 * @file motors.h
 * @brief Управление 4 частотными приводами (2 горизонталь + 2 вертикаль) через Modbus.
 *
 * Принцип:
 *  - Приводам пишем:
 *      MB_REG_CMD (0001H): forward/reverse/stop/reset
 *      MB_REG_SETPOINT (0002H): -10000..10000 (=-100.00..100.00%)
 *  - Команды отправляются не чаще MOTORS_TICK_MS и только при изменениях.
 *
 * ВНИМАНИЕ по направлению:
 *  - Для каждого привода можно инвертировать направление в config (см. DriveMap).
 */
#pragma once
#include <stdint.h>
#include "config.h"
#include "modbus.h"

enum class DriveId : uint8_t { H1=0, H2=1, V1=2, V2=3, COUNT=4 };

struct DriveMap {
  uint8_t addr;
  bool invertDir;   // если true, меняем местами forward/reverse (удобно при перепутанном подключении)
};

struct DriveTelemetry {
  // HE200 monitoring (D0.xx)
  // Частоты приходят в 0.01 Hz (например 1396 -> 13.96 Hz)
  uint16_t runFreq01Hz = 0;   // 0x7000
  uint16_t setFreq01Hz = 0;   // 0x7001
  uint16_t busV01V    = 0;   // 0x7002 (0.1 V DC bus)
  uint16_t faultInfo   = 0;   // 0x702D (0 = OK)
  uint16_t runState    = 0;   // 0x703D

  bool connected = false;
  uint8_t lastErr = 0;     // 0=ok, 1=timeout, 2=crc, 3=exception, 4=bad_response
  uint32_t lastOkMs = 0;   // когда последний раз получили валидный ответ
  uint8_t regMode = 0;     // 0=unknown,1=03,2=04,3=03@(base-1),4=04@(base-1)
};

class Drives {
public:
  void begin(ModbusMasterRTU& mb);

  // speedPct: -100..100
  //   знак = направление КОМАНДЫ частотнику:
  //     + => Forward
  //     - => Reverse
  //   а что такое Forward/Reverse по механике задаётся вашими словами:
  //     H Forward = вправо
  //     V Forward = вниз
  //   (см. config.h)
  void setSpeed(DriveId id, int16_t speedPct);
  void stop(DriveId id);
  void stopAll();

  // Обновляет физические команды (отправка Modbus) и опрос телеметрии (редко)
  void tick(uint32_t nowMs);

  // Мягкий сброс ошибок частотника
  void resetFault(DriveId id);

  const DriveTelemetry& telemetry(DriveId id) const { return _tel[(uint8_t)id]; }

private:
  ModbusMasterRTU* _mb = nullptr;

  // Round-robin индексы, чтобы не блокировать loop кучей Modbus-запросов подряд.
  uint8_t _rrSend = 0;
  uint8_t _rrPoll = 0;

  struct DriveState {
    int16_t targetPct = 0;
    int16_t sentPct   = 0;
    uint32_t lastSend = 0;
    uint32_t lastPoll = 0;
    uint32_t lastDiag = 0;
    uint8_t  diagPhase = 0; // 0=fault, 1=state
    uint8_t  regMode = 0;  // 0=unknown,1=03,2=04,3=03@(base-1),4=04@(base-1)
    // Авто-пробник для regMode==0. Вместо 4 запросов подряд (что может
    // «подвешивать» UI при отсутствии привода) пробуем по одному режиму
    // за тик: 0..3 → (03),(04),(03 base-1),(04 base-1).
    uint8_t  probePhase = 0;
    uint8_t  failStreak = 0;
    bool     needStopCmd = false;
  };

  DriveState _st[(uint8_t)DriveId::COUNT];
  DriveTelemetry _tel[(uint8_t)DriveId::COUNT];

  DriveMap _map[(uint8_t)DriveId::COUNT];
  void sendCommand(DriveId id, int16_t pct);
  void pollTelemetry(DriveId id, uint32_t nowMs);
};

