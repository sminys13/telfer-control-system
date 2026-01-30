#include <Arduino.h>
#include "app.h"

// Один глобальный экземпляр приложения.
static App g_app;

void setup() {
  g_app.setup();
}

void loop() {
  g_app.loop();
}
