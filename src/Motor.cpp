// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "V2Stepper.h"

void V2Stepper::Motor::begin() {
  Driver::begin();
  reset();
}

void V2Stepper::Motor::reset() {
  Driver::reset();

  _mode     = Mode::Idle;
  _high     = false;
  _reverse  = false;
  _position = {};
  _speed    = {};
  _queue    = {};

  setCurrentScale(config.ampere);

  // Motor run current.
  tmc2130_writeField(&_tmc2130, TMC2130_IRUN_FIELD, _currentScale);

  // Motor standstill to motor current power down period.
  tmc2130_writeField(&_tmc2130, TMC2130_TPOWERDOWN_FIELD, 10);

  // Motor standstill current.
  tmc2130_writeField(&_tmc2130, TMC2130_IHOLD_FIELD, 0);

  // Motor standstill to powerdown period, after TPOWERDOWN.
  tmc2130_writeField(&_tmc2130, TMC2130_IHOLDDELAY_FIELD, 6);

  // Stand-still option: Freewheeling, applies if IHOLD == 0.
  tmc2130_writeField(&_tmc2130, TMC2130_FREEWHEEL_FIELD, 2);

  // Number of microsteps.
  tmc2130_writeField(&_tmc2130, TMC2130_MRES_FIELD, 8 - config.microstepsShift);

  // Interpolate to 256 microsteps.
  tmc2130_writeField(&_tmc2130, TMC2130_INTPOL_FIELD, 1);

  // Off time and driver enable.
  tmc2130_writeField(&_tmc2130, TMC2130_TOFF_FIELD, 3);

  // Hysteresis start value.
  tmc2130_writeField(&_tmc2130, TMC2130_HSTRT_FIELD, 4);

  // Hysteresis low value.
  tmc2130_writeField(&_tmc2130, TMC2130_HEND_FIELD, 1);

  // Blank time select.
  tmc2130_writeField(&_tmc2130, TMC2130_TBL_FIELD, 2);

  // Chopper mode.
  tmc2130_writeField(&_tmc2130, TMC2130_CHM_FIELD, 0);

  // PWM frequency selection.
  tmc2130_writeField(&_tmc2130, TMC2130_PWM_FREQ_FIELD, 0);

  // User defined amplitude.
  tmc2130_writeField(&_tmc2130, TMC2130_PWM_GRAD_FIELD, 1);

  // PWM automatic amplitude scaling.
  tmc2130_writeField(&_tmc2130, TMC2130_PWM_AUTOSCALE_FIELD, 1);

  // User defined PWM amplitude offset.
  tmc2130_writeField(&_tmc2130, TMC2130_PWM_AMPL_FIELD, 200);

  // Upper velocity for stealthChop voltage PWM mode.
  tmc2130_writeField(&_tmc2130, TMC2130_TPWMTHRS_FIELD, 500);

  // StealthChop voltage PWM mode enabled.
  tmc2130_writeField(&_tmc2130, TMC2130_EN_PWM_MODE_FIELD, 1);

  // Stall guard threshold.
  const int8_t threshold = (float)INT8_MAX * config.home.stall;
  tmc2130_writeField(&_tmc2130, TMC2130_SGT_FIELD, threshold >> 1);
}

void V2Stepper::Motor::freewheel() {
  tmc2130_writeField(&_tmc2130, TMC2130_IHOLD_FIELD, 0);
  tmc2130_writeField(&_tmc2130, TMC2130_IHOLDDELAY_FIELD, 0);
  tmc2130_writeField(&_tmc2130, TMC2130_FREEWHEEL_FIELD, 1);
}

void V2Stepper::Motor::hold(float fraction) {
  const float cs = (float)_currentScale * fraction;
  tmc2130_writeField(&_tmc2130, TMC2130_IHOLD_FIELD, ceilf(cs));
}

void V2Stepper::Motor::setDirection(bool reverse) {
  tmc2130_writeField(&_tmc2130, TMC2130_SHAFT_FIELD, reverse ^ config.inverse);
}

void V2Stepper::Motor::loop() {
  switch (_mode) {
    case Mode::HomeStall:
      if ((unsigned long)(micros() - _usec) < 100)
        return;

      _usec = micros();

      // Always move a few steps before checking the 'stall' flag.
      if (_position.now >= (uint8_t)(1 << config.microstepsShift) * 8 &&
          tmc2130_readField(&_tmc2130, TMC2130_STALLGUARD_FIELD)) {
        _timer->disable();
        _position.now    = _position.start;
        _position.target = 0;

        // Move forward, but count downwards to zero.
        _reverse = true;
        setDirection(false);

        init(Mode::Home);
        _timer->enable();
        handleMovement(Move::Forward);
      }
      break;

    case Mode::Done:
      _mode = Mode::Idle;
      handleMovement(Move::Stop);

      switch (_queue.mode) {
        case Mode::Position:
          position(_queue.position.target, _queue.position.speedFraction, _queue.position.handler);
          _queue = {};
          break;

        case Mode::HomeStall:
          home(_queue.home.steps, _queue.home.start, _queue.home.handler);
          _queue = {};
          break;

        case Mode::Rotate:
          rotate(_queue.rotate.speedFraction);
          _queue = {};
          break;
      }

      if (_position.handler)
        _position.handler();
      break;
  }
}

void V2Stepper::Motor::tick() {
  // Reset step signal back to logical low.
  if (_high) {
    _high = false;
    _pinStep.low();
  }

  if (_mode == Mode::Idle || _mode == Mode::Done)
    return;

  // Speed calculation, modulates the pulse frequency.
  if (_adjustCounter == 0) {
    _adjustCounter = _tickFrequency / _adjustmentFrequency;

    if (_speed.now < _speed.target) {
      _speed.now += _speedAdjust;
      if (_speed.now > _speed.target)
        _speed.now = _speed.target;

    } else if (_speed.now > _speed.target) {
      _speed.now -= _speedAdjust;
      if (_speed.now < _speed.target)
        _speed.now = _speed.target;
    }

    _pulse.period = _tickFrequency / _speed.now;
  }
  _adjustCounter--;

  // Pulse / stepping signal frequency.
  if (_pulse.counter == 0) {
    _pulse.counter = _pulse.period;

    switch (_mode) {
      case Mode::HomeStall:
      case Mode::Home:
      case Mode::Position:
        if (_position.getDistance() == _position.decel)
          _speed.target = config.speed.min * (1 << config.microstepsShift);

        if (_position.now == _position.target) {
          _mode = Mode::Done;
          return;
        }

        if (_reverse)
          _position.now--;

        else
          _position.now++;
        break;

      case Mode::Rotate:
        if ((uint32_t)_speed.now < config.speed.min * (uint8_t)(1 << config.microstepsShift)) {
          _mode = Mode::Done;
          return;
        }
        break;
    }

    // Switch step signal to logical high.
    _high = true;
    _pinStep.high();
  }

  _pulse.counter--;
}

void V2Stepper::Motor::init(Mode mode) {
  _mode           = mode;
  _speed.now      = config.speed.min * (1 << config.microstepsShift);
  _pulse.counter  = 0;
  _adjustCounter = 0;
}

uint32_t V2Stepper::Motor::getMaxSpeed(float speedFraction) {
  const float range    = config.speed.max - config.speed.min;
  const uint32_t speed = config.speed.min + (range * speedFraction);
  return speed * (1 << config.microstepsShift);
}

uint32_t V2Stepper::Motor::getAccelerationSteps(uint32_t speedFrom, uint32_t speedTo) {
  const int32_t from   = speedFrom * speedFrom;
  const int32_t to     = speedTo * speedTo;
  const uint32_t accel = abs(to - from);
  return accel / (config.speed.accel * (1 << config.microstepsShift) * 2);
}

void V2Stepper::Motor::calculateDecelerationSteps() {
  const uint32_t accel = getAccelerationSteps(_speed.now, _speed.target);
  const uint32_t decel = getAccelerationSteps(_speed.target, getMaxSpeed(0));

  // Are we able to reach the full speed?
  if (accel + decel > _position.getDistance()) {
    const uint32_t delta = (accel + decel) - _position.getDistance();
    _position.decel      = decel - (delta / 2);

  } else
    _position.decel = decel;
}

void V2Stepper::Motor::position(float position, float speedFraction, void (*handler)()) {
  switch (_mode) {
    case Mode::HomeStall:
    case Mode::Home:
      _queue = {
        .mode{Mode::Position},
        .position{.target{position}, .speedFraction{speedFraction}, .handler{handler}},
      };
      break;

    case Mode::Idle: {
      const uint32_t target = position * (float)(1 << config.microstepsShift);
      if (target == _position.now) {
        if (handler)
          handler();

        break;
      }

      _speed.now        = config.speed.min * (1 << config.microstepsShift);
      _position.target  = target;
      _position.handler = handler;
      _speed.target     = getMaxSpeed(speedFraction);
      calculateDecelerationSteps();

      _reverse = _position.getReverse(target);
      setDirection(_reverse);
      init(Mode::Position);
      handleMovement(_reverse ? Move::Reverse : Move::Forward);
      break;
    }

    case Mode::Position: {
      const uint32_t target = position * (float)(1 << config.microstepsShift);
      _timer->disable();
      const bool reverse = _position.getReverse(target);

      // Do we need to reverse the direction?
      if (_mode == Mode::Position && _reverse != reverse) {
        stopPositioning();
        _queue = {
          .mode{Mode::Position},
          .position{.target{position}, .speedFraction{speedFraction}},
        };
        _timer->enable();
        break;
      }

      _position.target = target;
      _speed.target    = getMaxSpeed(speedFraction);
      calculateDecelerationSteps();

      // If we are already too fast to stop at the target, overshoot and move back.
      if (_position.getDistance() < _position.decel) {
        stopPositioning();
        _queue = {
          .mode{Mode::Position},
          .position{.target{position}, .speedFraction{speedFraction}},
        };
      }

      _timer->enable();
      break;
    }
  }
}

void V2Stepper::Motor::stopPositioning() {
  _speed.target = config.speed.min * (1 << config.microstepsShift);

  const uint32_t decel = getAccelerationSteps(_speed.now, getMaxSpeed(0));
  if (_reverse)
    _position.target = _position.now - decel;

  else
    _position.target = _position.now + decel;
}

void V2Stepper::Motor::stop() {
  switch (_mode) {
    case Mode::Position:
      _queue = {};
      _timer->disable();
      stopPositioning();
      _timer->enable();
      break;

    case Mode::Rotate:
      rotate(0);
      break;
  }
}

void V2Stepper::Motor::home(uint32_t steps, uint32_t start, void (*handler)()) {
  switch (_mode) {
    case Mode::HomeStall:
    case Mode::Home:
      break;

    case Mode::Idle:
      // Move backwards, but count upwards to target.
      _reverse = false;
      setDirection(true);

      _position = {
        .target{steps * (1 << config.microstepsShift)},
        .start{start * (1 << config.microstepsShift)},
        .handler{handler},
      };
      _speed.target = config.home.speed * (1 << config.microstepsShift);
      handleMovement(Move::Reverse);
      init(Mode::HomeStall);
      break;

    default:
      stop();
      _queue = {
        .mode{Mode::HomeStall},
        .home{.steps{steps}, .start{start}, .handler{handler}},
      };
      break;
  }
}

void V2Stepper::Motor::rotate(float speedFraction) {
  if (_mode != Mode::Idle && _mode != Mode::Rotate)
    return;

  const bool reverse = speedFraction < 0;

  // Change direction.
  if (_mode == Mode::Rotate && _reverse != reverse) {
    _queue = {
      .mode{Mode::Rotate},
      .rotate{.speedFraction{speedFraction}, .reverse{reverse}},
    };
    _speed.target = 0;
    return;
  }

  _queue = {};

  if (fabs(speedFraction) < 0.00001f)
    _speed.target = 0;

  else
    _speed.target = getMaxSpeed(fabs(speedFraction));

  if (_mode == Mode::Idle) {
    _reverse = reverse;
    setDirection(_reverse);
    handleMovement(_reverse ? Move::Reverse : Move::Forward);
    init(Mode::Rotate);
  }
}
