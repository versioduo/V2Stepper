// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "V2Stepper.h"

void V2Stepper::Power::begin() {
  Driver::begin();
  reset();
}

void V2Stepper::Power::reset() {
  Driver::reset();

  // Motor coil currents and polarity directly programmed via serial interface.
  tmc2130_writeField(&_tmc2130, TMC2130_DIRECT_MODE_FIELD, 1);

  // Motor coil current / velocity.
  setCurrentScale(config.ampere);

  // Motor standstill current.
  tmc2130_writeField(&_tmc2130, TMC2130_IHOLD_FIELD, _currentScale);

  // Off time and driver enable.
  tmc2130_writeField(&_tmc2130, TMC2130_TOFF_FIELD, 3);

  // User defined amplitude.
  tmc2130_writeField(&_tmc2130, TMC2130_PWM_GRAD_FIELD, 4);

  // PWM automatic amplitude scaling.
  tmc2130_writeField(&_tmc2130, TMC2130_PWM_AUTOSCALE_FIELD, 1);

  // User defined PWM amplitude offset.
  tmc2130_writeField(&_tmc2130, TMC2130_PWM_AMPL_FIELD, 255);

  // StealthChop voltage PWM mode enabled.
  tmc2130_writeField(&_tmc2130, TMC2130_EN_PWM_MODE_FIELD, 1);
}

void V2Stepper::Power::scaleVoltage(uint8_t channel, float fraction) {
  const int16_t scale = (float)INT16_MAX * fraction;

  switch (channel) {
    case 0:
      handleScaleVoltage(0, fraction);
      tmc2130_writeField(&_tmc2130, TMC2130_DIRECT_CURRENT_A_FIELD, scale >> 7);
      break;

    case 1:
      handleScaleVoltage(1, fraction);
      tmc2130_writeField(&_tmc2130, TMC2130_DIRECT_CURRENT_B_FIELD, scale >> 7);
      break;
  }
}
