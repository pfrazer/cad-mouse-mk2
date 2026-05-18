#pragma once

#include <Arduino.h>

#include "State.h"

class CalibratingState : public State {
 public:
  void enter() override;
  void update() override;
  void exit() override;

 private:
  unsigned long startTimeMS = 0;
  bool timedOut = false;
};
