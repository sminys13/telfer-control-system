#include <Arduino.h>
#include <SPI.h>
#include <string.h>

// =====================================================
// FIELD TEST: fast non-blocking SINGLE for X1/X2
// =====================================================

// ---------- USER SETTINGS ----------
static constexpr bool TEST_X1 = true;
static constexpr bool TEST_X2 = true;

// UART / DWIN
static constexpr uint32_t DBG_BAUD = 115200;
static constexpr uint32_t DWIN_BAUD = 115200;

// SPI slaves
static constexpr uint8_t SC16_1_CS_PIN = 10;
static constexpr uint8_t SC16_2_CS_PIN = 8; // IMPORTANT: keep HIGH even if unused

// DWIN VP map
static constexpr uint16_t VP_X1 = 0x1000;
static constexpr uint16_t VP_X2 = 0x1002;
static constexpr uint16_t VP_Z1 = 0x1004; // age1 in ms
static constexpr uint16_t VP_Z2 = 0x1006; // age2 in ms
static constexpr uint16_t VP_MODE = 0x1010;
static constexpr uint16_t VP_ERROR = 0x1012;

static constexpr uint16_t MODE_FIELD_TEST = 11;

// SC16 UART
static constexpr uint16_t SC16_DIV_9600 = 12;

// Timings
static constexpr uint32_t INIT_GAP_MS = 180;
static constexpr uint32_t SAMPLE_PERIOD_X1_MS = 100; // try 100 ms
static constexpr uint32_t SAMPLE_PERIOD_X2_MS = 100;
static constexpr uint32_t SAMPLE_SHIFT_X2_MS = 50;
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 160;
static constexpr uint32_t STALE_TIMEOUT_MS = 900;
static constexpr uint32_t DWIN_UPDATE_MS = 60;
static constexpr uint32_t PRINT_UPDATE_MS = 200;

// Commands
static const uint8_t CMD_LASER_ON[] = {0x80, 0x06, 0x05, 0x01, 0x74};
static const uint8_t CMD_SINGLE[] = {0x80, 0x06, 0x02, 0x78};

// =====================================================
// SC16 low-level
// =====================================================
enum class Sc16Channel : uint8_t
{
  A = 0,
  B = 1
};

static constexpr uint8_t REG_RHR_THR = 0x00;
static constexpr uint8_t REG_IER = 0x01;
static constexpr uint8_t REG_FCR_IIR = 0x02;
static constexpr uint8_t REG_LCR = 0x03;
static constexpr uint8_t REG_SPR = 0x07;
static constexpr uint8_t REG_TXLVL = 0x08;
static constexpr uint8_t REG_RXLVL = 0x09;
static constexpr uint8_t REG_EFCR = 0x0F;
static constexpr uint8_t REG_DLL = 0x00;
static constexpr uint8_t REG_DLH = 0x01;

uint8_t sc16MakeAddr(uint8_t reg, Sc16Channel ch, bool isRead)
{
  uint8_t a = 0;
  if (isRead)
    a |= 0x80;
  a |= (reg & 0x0F) << 3;
  a |= (static_cast<uint8_t>(ch) & 0x03) << 1;
  return a;
}

void sc16WriteReg(uint8_t csPin, Sc16Channel ch, uint8_t reg, uint8_t value)
{
  digitalWrite(csPin, LOW);
  SPI.transfer(sc16MakeAddr(reg, ch, false));
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
}

uint8_t sc16ReadReg(uint8_t csPin, Sc16Channel ch, uint8_t reg)
{
  digitalWrite(csPin, LOW);
  SPI.transfer(sc16MakeAddr(reg, ch, true));
  uint8_t v = SPI.transfer(0xFF);
  digitalWrite(csPin, HIGH);
  return v;
}

void sc16UartInit(uint8_t csPin, Sc16Channel ch, uint16_t divisor)
{
  sc16WriteReg(csPin, ch, REG_IER, 0x00);
  sc16WriteReg(csPin, ch, REG_FCR_IIR, 0x07);

  sc16WriteReg(csPin, ch, REG_LCR, 0x80); // DLAB=1
  sc16WriteReg(csPin, ch, REG_DLL, (uint8_t)(divisor & 0xFF));
  sc16WriteReg(csPin, ch, REG_DLH, (uint8_t)(divisor >> 8));

  sc16WriteReg(csPin, ch, REG_LCR, 0x03); // 8N1
  sc16WriteReg(csPin, ch, REG_EFCR, 0x00);
}

bool sc16SelfTest(uint8_t csPin, Sc16Channel ch)
{
  sc16WriteReg(csPin, ch, REG_SPR, 0x55);
  uint8_t a = sc16ReadReg(csPin, ch, REG_SPR);

  sc16WriteReg(csPin, ch, REG_SPR, 0xAA);
  uint8_t b = sc16ReadReg(csPin, ch, REG_SPR);

  return (a == 0x55) && (b == 0xAA);
}

bool sc16TxReady(uint8_t csPin, Sc16Channel ch)
{
  return sc16ReadReg(csPin, ch, REG_TXLVL) > 0;
}

void sc16WriteByte(uint8_t csPin, Sc16Channel ch, uint8_t b)
{
  while (!sc16TxReady(csPin, ch))
  {
    // short busy wait only
  }
  sc16WriteReg(csPin, ch, REG_RHR_THR, b);
}

void sc16WriteBuf(uint8_t csPin, Sc16Channel ch, const uint8_t *data, size_t len)
{
  for (size_t i = 0; i < len; ++i)
  {
    sc16WriteByte(csPin, ch, data[i]);
  }
}

int sc16ReadByte(uint8_t csPin, Sc16Channel ch)
{
  if (sc16ReadReg(csPin, ch, REG_RXLVL) == 0)
    return -1;
  return sc16ReadReg(csPin, ch, REG_RHR_THR);
}

// =====================================================
// DWIN helper
// =====================================================
void dwinWriteU16(uint16_t vp, uint16_t value)
{
  uint8_t frame[8];
  frame[0] = 0x5A;
  frame[1] = 0xA5;
  frame[2] = 0x05;
  frame[3] = 0x82;
  frame[4] = (uint8_t)(vp >> 8);
  frame[5] = (uint8_t)(vp & 0xFF);
  frame[6] = (uint8_t)(value >> 8);
  frame[7] = (uint8_t)(value & 0xFF);
  Serial2.write(frame, sizeof(frame));
}

// =====================================================
// Parser helpers
// =====================================================
uint8_t calcChecksum(const uint8_t *data, size_t lenWithoutChecksum)
{
  uint8_t sum = 0;
  for (size_t i = 0; i < lenWithoutChecksum; ++i)
  {
    sum = (uint8_t)(sum + data[i]);
  }
  return (uint8_t)(~sum + 1);
}

bool isAsciiDigitOrDot(uint8_t c)
{
  return (c >= '0' && c <= '9') || (c == '.');
}

// =====================================================
// Laser state
// =====================================================
enum class InitStage : uint8_t
{
  Start,
  WaitAfterLaserOn,
  Running
};

struct LaserChannelState
{
  uint8_t csPin;
  Sc16Channel ch;
  const char *name;
  uint16_t dwinVp;
  bool enabled;

  InitStage initStage;
  bool waitingReply;

  uint32_t nextActionMs;
  uint32_t replyDeadlineMs;
  uint32_t lastValidMs;

  bool valid;
  int32_t mm;

  uint8_t rxBuf[64];
  size_t rxLen;
};

LaserChannelState g_x1;
LaserChannelState g_x2;

void initLaserState(LaserChannelState &s,
                    uint8_t csPin,
                    Sc16Channel ch,
                    const char *name,
                    uint16_t vp,
                    bool enabled)
{
  s.csPin = csPin;
  s.ch = ch;
  s.name = name;
  s.dwinVp = vp;
  s.enabled = enabled;

  s.initStage = InitStage::Start;
  s.waitingReply = false;
  s.nextActionMs = 0;
  s.replyDeadlineMs = 0;
  s.lastValidMs = 0;
  s.valid = false;
  s.mm = 0;
  s.rxLen = 0;
}

void flushRx(LaserChannelState &s)
{
  while (true)
  {
    int b = sc16ReadByte(s.csPin, s.ch);
    if (b < 0)
      break;
  }
  s.rxLen = 0;
}

void appendRxByte(LaserChannelState &s, uint8_t b)
{
  if (s.rxLen < sizeof(s.rxBuf))
  {
    s.rxBuf[s.rxLen++] = b;
    return;
  }

  memmove(s.rxBuf, s.rxBuf + 1, sizeof(s.rxBuf) - 1);
  s.rxBuf[sizeof(s.rxBuf) - 1] = b;
}

// returns true if valid measurement extracted
bool tryExtractMeasurement(LaserChannelState &s, int32_t &outMm)
{
  // First strip ACK frames: 80 06 85 01 F4
  for (size_t i = 0; i + 5 <= s.rxLen; ++i)
  {
    if (s.rxBuf[i + 0] == 0x80 &&
        s.rxBuf[i + 1] == 0x06 &&
        s.rxBuf[i + 2] == 0x85 &&
        s.rxBuf[i + 3] == 0x01 &&
        s.rxBuf[i + 4] == 0xF4)
    {
      const size_t consumed = i + 5;
      memmove(s.rxBuf, s.rxBuf + consumed, s.rxLen - consumed);
      s.rxLen -= consumed;
      return false;
    }
  }

  // Then search measurement frame:
  // 80 06 82 d d d . d d d chk
  // or 80 06 83 ...
  for (size_t i = 0; i + 11 <= s.rxLen; ++i)
  {
    if (s.rxBuf[i + 0] != 0x80)
      continue;
    if (s.rxBuf[i + 1] != 0x06)
      continue;
    if (!(s.rxBuf[i + 2] == 0x82 || s.rxBuf[i + 2] == 0x83))
      continue;

    bool asciiOk = true;
    for (size_t k = 3; k <= 9; ++k)
    {
      if (!isAsciiDigitOrDot(s.rxBuf[i + k]))
      {
        asciiOk = false;
        break;
      }
    }
    if (!asciiOk)
      continue;

    uint8_t chk = calcChecksum(&s.rxBuf[i], 10);
    if (chk != s.rxBuf[i + 10])
      continue;

    int metersInt =
        (s.rxBuf[i + 3] - '0') * 100 +
        (s.rxBuf[i + 4] - '0') * 10 +
        (s.rxBuf[i + 5] - '0');

    int fracMm =
        (s.rxBuf[i + 7] - '0') * 100 +
        (s.rxBuf[i + 8] - '0') * 10 +
        (s.rxBuf[i + 9] - '0');

    outMm = metersInt * 1000 + fracMm;

    const size_t consumed = i + 11;
    memmove(s.rxBuf, s.rxBuf + consumed, s.rxLen - consumed);
    s.rxLen -= consumed;
    return true;
  }

  // Trim old garbage if needed
  if (s.rxLen > 48)
  {
    memmove(s.rxBuf, s.rxBuf + (s.rxLen - 12), 12);
    s.rxLen = 12;
  }

  return false;
}

void serviceRx(LaserChannelState &s)
{
  if (!s.enabled)
    return;

  while (true)
  {
    int b = sc16ReadByte(s.csPin, s.ch);
    if (b < 0)
      break;
    appendRxByte(s, (uint8_t)b);
  }

  bool progress = true;
  while (progress)
  {
    progress = false;
    size_t oldLen = s.rxLen;

    int32_t parsedMm = 0;
    if (tryExtractMeasurement(s, parsedMm))
    {
      if (parsedMm > 0)
      {
        s.mm = parsedMm;
        s.valid = true;
        s.lastValidMs = millis();
        s.waitingReply = false;
      }
      progress = true;
    }
    else if (s.rxLen != oldLen)
    {
      progress = true;
    }
  }

  if (s.valid && (millis() - s.lastValidMs > STALE_TIMEOUT_MS))
  {
    s.valid = false;
  }
}

void initStateMachine(LaserChannelState &s, uint32_t now)
{
  if (!s.enabled)
    return;

  switch (s.initStage)
  {
  case InitStage::Start:
    if (now >= s.nextActionMs)
    {
      flushRx(s);
      sc16WriteBuf(s.csPin, s.ch, CMD_LASER_ON, sizeof(CMD_LASER_ON));
      s.initStage = InitStage::WaitAfterLaserOn;
      s.nextActionMs = now + INIT_GAP_MS;
    }
    break;

  case InitStage::WaitAfterLaserOn:
    if (now >= s.nextActionMs)
    {
      s.initStage = InitStage::Running;
      s.nextActionMs = now + 40;
    }
    break;

  case InitStage::Running:
    break;
  }
}

void pollSingle(LaserChannelState &s, uint32_t now, uint32_t periodMs)
{
  if (!s.enabled)
    return;
  if (s.initStage != InitStage::Running)
    return;

  if (s.waitingReply)
  {
    if (now >= s.replyDeadlineMs)
    {
      s.waitingReply = false;
    }
    return;
  }

  if (now >= s.nextActionMs)
  {
    flushRx(s);
    sc16WriteBuf(s.csPin, s.ch, CMD_SINGLE, sizeof(CMD_SINGLE));
    s.waitingReply = true;
    s.replyDeadlineMs = now + RESPONSE_TIMEOUT_MS;
    s.nextActionMs = now + periodMs;
  }
}

// =====================================================
// Arduino
// =====================================================
void setup()
{
  Serial.begin(DBG_BAUD);
  Serial2.begin(DWIN_BAUD);

  // Release all SPI slaves
  pinMode(53, OUTPUT);

  pinMode(SC16_1_CS_PIN, OUTPUT);
  digitalWrite(SC16_1_CS_PIN, HIGH);

  pinMode(SC16_2_CS_PIN, OUTPUT);
  digitalWrite(SC16_2_CS_PIN, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  initLaserState(g_x1, SC16_1_CS_PIN, Sc16Channel::A, "X1", VP_X1, TEST_X1);
  initLaserState(g_x2, SC16_1_CS_PIN, Sc16Channel::B, "X2", VP_X2, TEST_X2);

  Serial.println();
  Serial.println(F("========== FAST SINGLE FIELD TEST X1/X2 START =========="));

  dwinWriteU16(VP_MODE, MODE_FIELD_TEST);
  dwinWriteU16(VP_ERROR, 0);
  dwinWriteU16(VP_X1, 0);
  dwinWriteU16(VP_X2, 0);
  dwinWriteU16(VP_Z1, 0);
  dwinWriteU16(VP_Z2, 0);

  sc16UartInit(SC16_1_CS_PIN, Sc16Channel::A, SC16_DIV_9600);
  sc16UartInit(SC16_1_CS_PIN, Sc16Channel::B, SC16_DIV_9600);

  Serial.print(F("SC16 #1 CH_A: "));
  Serial.println(sc16SelfTest(SC16_1_CS_PIN, Sc16Channel::A) ? F("OK") : F("FAIL"));

  Serial.print(F("SC16 #1 CH_B: "));
  Serial.println(sc16SelfTest(SC16_1_CS_PIN, Sc16Channel::B) ? F("OK") : F("FAIL"));

  uint32_t now = millis();
  g_x1.nextActionMs = now + 100;
  g_x2.nextActionMs = now + 100 + SAMPLE_SHIFT_X2_MS;
}

void loop()
{
  const uint32_t now = millis();

  serviceRx(g_x1);
  serviceRx(g_x2);

  initStateMachine(g_x1, now);
  initStateMachine(g_x2, now);

  pollSingle(g_x1, now, SAMPLE_PERIOD_X1_MS);
  pollSingle(g_x2, now, SAMPLE_PERIOD_X2_MS);

  // fast DWIN update
  static uint32_t lastDwinMs = 0;
  if (now - lastDwinMs >= DWIN_UPDATE_MS)
  {
    lastDwinMs = now;

    dwinWriteU16(VP_X1, g_x1.valid ? (uint16_t)g_x1.mm : 0);
    dwinWriteU16(VP_X2, g_x2.valid ? (uint16_t)g_x2.mm : 0);

    uint16_t age1 = g_x1.valid ? (uint16_t)min(now - g_x1.lastValidMs, 9999UL) : 9999;
    uint16_t age2 = g_x2.valid ? (uint16_t)min(now - g_x2.lastValidMs, 9999UL) : 9999;

    dwinWriteU16(VP_Z1, age1);
    dwinWriteU16(VP_Z2, age2);

    uint16_t err = 0;
    if (TEST_X1 && !g_x1.valid)
      err |= 1;
    if (TEST_X2 && !g_x2.valid)
      err |= 2;

    dwinWriteU16(VP_MODE, MODE_FIELD_TEST);
    dwinWriteU16(VP_ERROR, err);
  }

  // serial diagnostics
  static uint32_t lastPrintMs = 0;
  if (now - lastPrintMs >= PRINT_UPDATE_MS)
  {
    lastPrintMs = now;

    Serial.print(F("X1="));
    if (g_x1.valid)
    {
      Serial.print(g_x1.mm);
      Serial.print(F("mm"));
    }
    else
    {
      Serial.print(F("---"));
    }

    Serial.print(F(" | X2="));
    if (g_x2.valid)
    {
      Serial.print(g_x2.mm);
      Serial.print(F("mm"));
    }
    else
    {
      Serial.print(F("---"));
    }

    Serial.print(F(" | age1="));
    Serial.print(g_x1.valid ? (now - g_x1.lastValidMs) : 9999);
    Serial.print(F("ms"));

    Serial.print(F(" | age2="));
    Serial.print(g_x2.valid ? (now - g_x2.lastValidMs) : 9999);
    Serial.println(F("ms"));
  }
}