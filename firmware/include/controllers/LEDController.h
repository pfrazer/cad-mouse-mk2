#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

class LEDController {
 public:
  LEDController();
  void begin();
  void setSolid(unsigned long color);
  void showMotion(const float motion[6]);
  void startSpinner(unsigned long color);
  void updateSpinner();
  void off();

 private:
  enum class Mode {
    Off,
    Solid,
    Motion,
    Spinner,
  };

  void setPower(bool enabled);
  void fillAll(unsigned long color);
  unsigned long toNeoColor(unsigned long color);
  static float clampf(float value, float lo, float hi);
  static int colorComponent(unsigned long color, int shift);
  unsigned long blendMotionColor(unsigned long fromColor, unsigned long toColor,
                                 float amount);

  bool isPowered_ = false;
  Mode mode_ = Mode::Off;
  unsigned long color_ = 0;
  int spinnerIndex_ = 0;
  unsigned long lastSpinnerStepMs_ = 0;
  Adafruit_NeoPixel ring_;
};
