#include "keypad.h"

// Key mapping stored in flash (PROGMEM) to keep RAM usage minimal.
const char Keypad4x4::MAP[4][4] PROGMEM = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};


// Key mapping is defined below as Keypad4x4::MAP (PROGMEM).

// Пины rows/cols берём из config.h
static const uint8_t KP_COLS[4] = { PIN_KP_C1, PIN_KP_C2, PIN_KP_C3, PIN_KP_C4 };
static const uint8_t KP_ROWS[4] = { PIN_KP_R1, PIN_KP_R2, PIN_KP_R3, PIN_KP_R4 };

static inline uint8_t bitIndex(uint8_t row, uint8_t col) { return (uint8_t)(row*4 + col); }

void Keypad4x4::begin() {
  // Колонки — выходы, по умолчанию HIGH.
  for (uint8_t c=0;c<4;c++) {
    pinMode(KP_COLS[c], OUTPUT);
    digitalWrite(KP_COLS[c], HIGH);
  }
  // Ряды — входы с подтяжкой (внутренний pull-up в Mega).
  for (uint8_t r=0;r<4;r++) {
    pinMode(KP_ROWS[r], INPUT_PULLUP);
  }
  _rawMask = 0;
  _stableMask = 0;
  _prevStableMask = 0;
  _rawChangeMs = 0;
  _eventKey = 0;
}

uint16_t Keypad4x4::scanRawMask() const {
  uint16_t mask = 0;

  // Важно: всегда держим только одну колонку в LOW.
  // Это стандартный метод сканирования.
  for (uint8_t c=0;c<4;c++) {
    // все HIGH
    for (uint8_t cc=0; cc<4; cc++) digitalWrite(KP_COLS[cc], HIGH);
    // текущую LOW
    digitalWrite(KP_COLS[c], LOW);
    delayMicroseconds(5);

    for (uint8_t r=0;r<4;r++) {
      // Ряд нажатой клавиши станет LOW.
      bool down = (digitalRead(KP_ROWS[r]) == LOW);
      if (down) {
        mask |= (uint16_t)(1u << bitIndex(r,c));
      }
    }
  }

  // Вернём колонки в HIGH (чтобы не "кормить" паразитные токи через нажатые клавиши).
  for (uint8_t cc=0; cc<4; cc++) digitalWrite(KP_COLS[cc], HIGH);

  return mask;
}

char Keypad4x4::maskBitToChar(uint8_t bit) {
  uint8_t row = bit / 4;
  uint8_t col = bit % 4;
  if (row >= 4 || col >= 4) return 0;
  return (char)pgm_read_byte(&MAP[row][col]);
}

void Keypad4x4::tick(uint32_t nowMs) {
  const uint16_t raw = scanRawMask();

  if (raw != _rawMask) {
    _rawMask = raw;
    _rawChangeMs = nowMs;
  }

  // Дребезг: считаем состояние стабильным, если оно не менялось 30 мс.
  if ((uint32_t)(nowMs - _rawChangeMs) < 30) return;

  if (raw == _stableMask) return;

  uint16_t prev = _stableMask;
  _stableMask = raw;

  // Событие только на нажатие (переход 0->1)
  uint16_t pressed = (uint16_t)(_stableMask & ~prev);
  if (pressed != 0 && _eventKey == 0) {
    // Берём первый бит (самый младший) — этого достаточно для наших задач.
    for (uint8_t b=0;b<16;b++) {
      if (pressed & (1u << b)) {
        _eventKey = maskBitToChar(b);
        break;
      }
    }
  }
}

char Keypad4x4::popKey() {
  char k = _eventKey;
  _eventKey = 0;
  return k;
}

bool Keypad4x4::isDown(char key) const {
  // Проходим по карте и ищем соответствие.
  for (uint8_t r=0;r<4;r++) {
    for (uint8_t c=0;c<4;c++) {
      if ((char)pgm_read_byte(&MAP[r][c]) == key) {
        uint8_t b = bitIndex(r,c);
        return (_stableMask & (1u << b)) != 0;
      }
    }
  }
  return false;
}