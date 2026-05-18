#include "states/SleepState.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void SleepState::enter() {
  ledController.off();
  lastKnobPollMS = millis();
}

void SleepState::update() {
  inputController.update();

  if (inputController.takeActivity() || knobWasMoved()) {
    stateMachine.changeState(&StateMachine::idleState);
    return;
  }
}

void SleepState::exit() {}

bool SleepState::knobWasMoved() {
  const unsigned long now = millis();
  if ((now - lastKnobPollMS) < Config::SLEEP_KNOB_POLL_PERIOD_MS) {
    return false;
  }
  lastKnobPollMS = now;

  float raw[9] = {};
  if (!sensorController.readRaw(raw)) {
    return false;
  }

  const float* baseline = sensorController.baseline();
  for (int i = 0; i < 9; i++) {
    if (fabsf(raw[i] - baseline[i]) > Config::SLEEP_KNOB_MOVEMENT_THRESHOLD) {
      return true;
    }
  }
  return false;
}
