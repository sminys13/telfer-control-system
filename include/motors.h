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
  uint16_t statusReg;  // 0x0020
  uint16_t faultCode;  // 0x0021
  bool connected;
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

  struct DriveState {
    int16_t targetPct = 0;
    int16_t sentPct   = 0;
    uint32_t lastSend = 0;
    uint32_t lastPoll = 0;
    bool     needStopCmd = false;
  };

  DriveState _st[(uint8_t)DriveId::COUNT];
  DriveTelemetry _tel[(uint8_t)DriveId::COUNT];

  DriveMap _map[(uint8_t)DriveId::COUNT];

  void sendCommand(DriveId id, int16_t pct);
  void pollTelemetry(DriveId id);
};

