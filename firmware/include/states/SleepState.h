#pragma once

#include <Arduino.h>

#include "State.h"

class SleepState : public State {
 public:
  void enter() override;
  void update() override;
  void exit() override;

 private:
  bool knobWasMoved();

  unsigned long lastKnobPollMS = 0;
};
