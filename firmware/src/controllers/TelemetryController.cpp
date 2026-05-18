#include "controllers/TelemetryController.h"
#include <Arduino.h>
#include "Config.h"

namespace {
const int kPrintEvery = 25;
}

void TelemetryController::begin() { tick_ = 0; }

bool TelemetryController::enabled() const { return Config::ENABLE_TELEMETRY; }

void TelemetryController::publish(const float motion[6], int buttonBits,
                                  bool hidReportSent) {
  if (!enabled()) {
    return;
  }

  tick_++;
  if ((tick_ % kPrintEvery) != 0) {
    return;
  }

  Serial.print("X:");
  Serial.print(motion[0]);
  Serial.print("  Y:");
  Serial.print(motion[1]);
  Serial.print("  Z:");
  Serial.print(motion[2]);
  Serial.print("    Rx:");
  Serial.print(motion[3]);
  Serial.print("  Ry:");
  Serial.print(motion[4]);
  Serial.print("  Rz:");
  Serial.print(motion[5]);
  Serial.print("    btn:");
  Serial.print(buttonBits & 0x0003);
  Serial.print("    hid:");
  Serial.println(hidReportSent ? 1 : 0);
}
