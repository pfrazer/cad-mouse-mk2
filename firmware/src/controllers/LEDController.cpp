#include "controllers/LEDController.h"

#include <math.h>

#include "Config.h"

namespace {
const int kAxisTx = 0;
const int kAxisTy = 1;
const int kAxisRx = 3;
const int kAxisRy = 4;

// LED positions, indexed D1..D8, as directional weights in knob X/Y space.
// D1 and D8 are top, D2/D3 left, D4/D5 bottom, and D6/D7 right.
const float kLedX[8] = {
    0.5f, 1.0f, 1.0f, 0.5f, -0.5f, -1.0f, -1.0f, -0.5f};
const float kLedY[8] = {
    -1.0f, -0.5f, 0.5f, 1.0f, 1.0f, 0.5f, -0.5f, -1.0f};
}  // namespace

LEDController::LEDController()
    : ring_(Config::LED_COUNT, Config::PIN_LED_DATA,
            NEO_GRB + NEO_KHZ800) {}

void LEDController::fillAll(unsigned long color) {
  for (int i = 0; i < ring_.numPixels(); i++) {
    ring_.setPixelColor(i, color);
  }
}

unsigned long LEDController::toNeoColor(unsigned long color) {
  int r = (color >> 16) & 0xFF;
  int g = (color >> 8) & 0xFF;
  int b = color & 0xFF;
  return ring_.Color(r, g, b);
}

float LEDController::clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

int LEDController::colorComponent(unsigned long color, int shift) {
  return (color >> shift) & 0xFF;
}

unsigned long LEDController::blendMotionColor(unsigned long fromColor,
                                              unsigned long toColor,
                                              float amount) {
  const float t = clampf(amount, 0.0f, 1.0f);
  const float idleBrightness = clampf(Config::LED_BRIGHTNESS / 255.0f,
                                      0.0f, 1.0f);

  const int fromR = colorComponent(fromColor, 16) * idleBrightness;
  const int fromG = colorComponent(fromColor, 8) * idleBrightness;
  const int fromB = colorComponent(fromColor, 0) * idleBrightness;
  const int toR = colorComponent(toColor, 16);
  const int toG = colorComponent(toColor, 8);
  const int toB = colorComponent(toColor, 0);

  const int r = fromR + (toR - fromR) * t;
  const int g = fromG + (toG - fromG) * t;
  const int b = fromB + (toB - fromB) * t;
  return ring_.Color(r, g, b);
}

void LEDController::begin() {
  pinMode(Config::PIN_LED_LS, OUTPUT);
  digitalWrite(Config::PIN_LED_LS, LOW);

  ring_.begin();
  ring_.setBrightness(Config::LED_BRIGHTNESS);
  ring_.show();
}

void LEDController::setPower(bool enabled) {
  if (enabled == isPowered_) {
    return;
  }

  isPowered_ = enabled;
  digitalWrite(Config::PIN_LED_LS, enabled ? HIGH : LOW);
  delay(10);
  
}

void LEDController::setSolid(unsigned long color) {
  mode_ = Mode::Solid;
  ring_.setBrightness(Config::LED_BRIGHTNESS);
  color_ = toNeoColor(color);
  setPower(true);
  fillAll(color_);
  ring_.show();
}

void LEDController::showMotion(const float motion[6]) {
  mode_ = Mode::Motion;
  setPower(true);
  ring_.setBrightness(255);

  // Translation and tilt both describe which way the knob is being moved.
  const float x = (motion[kAxisTx] * -1) + motion[kAxisRy];
  const float y = motion[kAxisTy] + motion[kAxisRx];
  const float magnitude = sqrtf(x * x + y * y);
  const float amount = clampf(magnitude / Config::AXIS_LIMIT, 0.0f, 1.0f);

  if (amount <= 0.0f) {
    fillAll(blendMotionColor(Config::LED_IDLE_COLOR,
                             Config::LED_INPUT_COLOR, 0.0f));
    ring_.show();
    return;
  }

  const float dirX = x / magnitude;
  const float dirY = y / magnitude;
  const int pixelCount = ring_.numPixels();
  for (int i = 0; i < pixelCount; i++) {
    const int ledIndex = i % 8;
    const float directionalWeight =
        clampf(dirX * kLedX[ledIndex] + dirY * kLedY[ledIndex], 0.0f, 1.0f);
    ring_.setPixelColor(
        i, blendMotionColor(Config::LED_IDLE_COLOR, Config::LED_INPUT_COLOR,
                            amount * directionalWeight));
  }
  ring_.show();
}

void LEDController::startSpinner(unsigned long color) {
  mode_ = Mode::Spinner;
  ring_.setBrightness(Config::LED_BRIGHTNESS);
  color_ = toNeoColor(color);
  spinnerIndex_ = 0;
  lastSpinnerStepMs_ = 0;
  setPower(true);
}

void LEDController::updateSpinner() {
  if (mode_ != Mode::Spinner) {
    return;
  }

  const unsigned long now = millis();
  if (lastSpinnerStepMs_ != 0 && (now - lastSpinnerStepMs_) < 60) {
    return;
  }
  lastSpinnerStepMs_ = now;

  fillAll(0);
  int pixelCount = ring_.numPixels();
  ring_.setPixelColor(spinnerIndex_, color_);
  ring_.show();

  spinnerIndex_++;
  if (spinnerIndex_ >= pixelCount) {
    spinnerIndex_ = 0;
  }
}

void LEDController::off() {
  mode_ = Mode::Off;
  fillAll(0);
  ring_.show();
  setPower(false);
}
