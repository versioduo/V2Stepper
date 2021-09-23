// © Kay Sievers <kay@vrfy.org>, 2020-2021
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <V2Base.h>

extern "C" {
#include "tmc/ic/TMC2130/TMC2130.h"
}

#include <Arduino.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <SPI.h>
#pragma GCC diagnostic pop

namespace V2Stepper {
class Driver {
public:
  struct Bus {
    SPIClass *spi;
    const uint8_t pin;
  };

  constexpr Driver(SPIClass *spi, uint8_t pin_select, uint8_t pin_step) :
    _bus({.spi{spi}, .pin{pin_select}}),
    _pin_step(pin_step) {}

protected:
  const Bus _bus;
  const uint8_t _pin_step;
  TMC2130TypeDef _tmc2130{};
  uint8_t _current_scale{};

  void begin();
  void reset();

  // Calculate the current scale factor. The sense resistor is adjusted according
  // to the configured current.
  void setCurrentScale(float ampere);
};

class Motor : public Driver {
public:
  const struct Config {
    // The motor coil current.
    float ampere;

    // The number of microsteps for one fullstep, specified as power of two. All
    // API step values still use fullsteps, but positioning accepts a fraction of
    // a step.
    uint8_t microsteps_shift;

    // The motor direction.
    bool inverse;

    // -1..1, a higher value makes StallGuard less sensitive.
    struct {
      // Speed for home movement in fillsteps per second.
      uint16_t speed;

      // Threshold for stall detection -1..1.
      float stall;
    } home;

    struct {
      // Speed limits in fullsteps per second.
      uint16_t min;
      uint16_t max;

      // Acceleration in fullsteps-per-second per second.
      uint16_t accel;
    } speed;
  } config;

  constexpr Motor(const Config conf,
                  V2Base::Timer::Periodic *timer,
                  SPIClass *spi,
                  uint8_t pin_select,
                  uint8_t pin_step) :
    Driver(spi, pin_select, pin_step),
    _pin_step(pin_step),
    _timer{timer},
    config(conf) {}

  void begin();
  void reset();

  // Critical sections disable the timer to not race against tick().
  void loop();

  bool isBusy() {
    return _mode != Mode::Idle;
  }

  // Periodic tick interrupt, expected to be called with a frequency
  // of 200kHz (_tick_frequency) from a timer interrupt.
  void tick();

  // Move to the given position. If microsteps are configured, the actual microstep
  // is determined by the fraction of the given fullstep value. The maximum speed of
  // the movemment is a fraction 0..1 of the configured maximum speed range.
  void position(float position, float speed_fraction = 1);

  // Apply a fraction of the configured current in stand-still mode to hold the motor
  // position. The default after a reset is free-wheeling with the coils shorted.
  void hold(float fraction = 0.1);

  float getPosition() {
    return (float)_position.now / (float)(1 << config.microsteps_shift);
  }

  // Stop the current movement.
  void stop();

  // Move at most the given number of fullsteps until the motor stalls, moves it the
  // number of 'start' fullsteps back, and sets the current position to zero.
  void home(uint32_t steps, uint32_t start);

  // Rotate at a given speed, The maximum speed of the movemment is a
  // fraction -1..1 the configured maximum speed range, 0 stops the rotation.
  void rotate(float speed_fraction);

protected:
  // Notify of changes, used to drive a status LED.
  enum class Move { Forward, Reverse, Stop };
  virtual void handleMovement(Move move){};

private:
  V2Base::Timer::Periodic *_timer;
  V2Base::GPIO _pin_step;
  static constexpr uint32_t _tick_frequency       = 200000;
  static constexpr uint32_t _adjustment_frequency = 200;
  enum class Mode { Idle, HomeStall, Home, Position, Rotate, Done } _mode{};
  unsigned long _usec{};

  // The current logic level of the step signal. The step signal will be logical high for the duration of one tick.
  bool _high{};

  // The stepping speed, a fraction of the 'tick()' frequency.
  struct {
    uint32_t counter;
    uint32_t period;
  } _pulse{};

  // The speed calculation frequency, a fraction of the 'tick()' frequency.
  uint32_t _adjust_counter{};

  // The current direction of the movement.
  bool _reverse{};

  struct {
    // The current position.
    uint32_t now;

    // The target position to move to.
    uint32_t target;

    // The number of steps needed to decelerate.
    uint32_t decel;

    // Number of steps to move backwards to reach zero after stall detection.
    uint32_t start;

    // Get the current number of steps to travel.
    uint32_t getDistance() {
      return abs((int32_t)target - (int32_t)now);
    }

    // check if the given position will need the direction of travel to be reversed.
    bool getReverse(uint32_t position) {
      const int32_t distance = (int32_t)position - (int32_t)now;
      return distance < 0;
    }
  } _position{};

  const uint32_t _speed_adjust = (float)(config.speed.accel * (1 << config.microsteps_shift)) / _adjustment_frequency;
  struct {
    float now;
    uint32_t target;
  } _speed{};

  struct {
    Mode mode;
    union {
      struct {
        float target;
        float speed_fraction;
        bool reverse;
      } position;

      struct {
        float speed_fraction;
        bool reverse;
      } rotate;

      struct {
        uint32_t steps;
        float speed_fraction;
        uint32_t start;
      } home;
    };
  } _queue{};

  void init(Mode mode);
  uint32_t getMaxSpeed(float speed_fraction);
  uint32_t getAccelerationSteps(uint32_t speed_from, uint32_t speed_to);
  void calculateDecelerationSteps();
  void setDirection(bool reverse);
  void stopPositioning();
};

// Configure the driver as a power supply, 2 channels, PWM voltage scaling. Used e.g. to
// drive two solenoids per driver.
class Power : public Driver {
public:
  const struct Config { float ampere; } config;
  constexpr Power(const Config &conf, SPIClass *spi, uint8_t pin_select, uint8_t pin_step) :
    Driver(spi, pin_select, pin_step),
    config(conf) {}
  void begin();
  void reset();

  // Sets the polarity and PWM duty cycle, 0..1.
  void scaleVoltage(uint8_t channel, float fraction);

protected:
  // Notify of changes, used to drive a status LED.
  virtual void handleScaleVoltage(uint8_t channel, float fraction){};
};
};
