#include "states/CalibratingState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void CalibratingState::enter() {
  sensorController.beginCalibration();
  motionController.reset();
  ledController.startSpinner(Config::LED_CALIBRATING_COLOR);
  startTimeMS = millis();
  timedOut = false;
}

void CalibratingState::update() {
  inputController.update();
  ledController.updateSpinner();
  sensorController.updateCalibration();

  // Keep activity from being published while calibrating.
  hidController.sendNeutralReport(0);

  if (sensorController.calibrationDone()) {
    stateMachine.changeState(&StateMachine::idleState);
    return;
  }

  // Set the LED ring to LED_ERROR_COLOR if the calibration takes longer
  // than CALIBRATION_TIMEOUT_MS to indicate a problem.  (potentially recoverable)  
  if (!timedOut && (millis() - startTimeMS) >= Config::CALIBRATION_TIMEOUT_MS) {
    ledController.setSolid(Config::LED_ERROR_COLOR);
    timedOut = true;
  }
}

void CalibratingState::exit() {}
