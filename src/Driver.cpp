// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "V2Stepper.h"

// Called from tmc/ic/TMC2130/TMC2130.c.
extern "C" {
void tmc2130_readWriteArray(uint8_t *data, size_t length, void *userData) {
  const V2Stepper::Driver::Bus *bus = (const V2Stepper::Driver::Bus *)userData;

  bus->spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  digitalWrite(bus->pin, LOW);
  bus->spi->transfer(data, length);
  digitalWrite(bus->pin, HIGH);
  bus->spi->endTransaction();
}
}

void V2Stepper::Driver::begin() {
  pinMode(_bus.pin, OUTPUT);
  digitalWrite(_bus.pin, HIGH);

  pinMode(_pinStep, OUTPUT);
  digitalWrite(_pinStep, LOW);

  tmc2130_init(&_tmc2130, tmc2130_defaultRegisterResetState, (void *)&_bus);
}

void V2Stepper::Driver::reset() {
  tmc2130_reset(&_tmc2130);
  while (tmc2130_periodicJob(&_tmc2130, 0))
    ;
}

// Motor coil current / velocity.
void V2Stepper::Driver::setCurrentScale(float ampere) {
  const float Rsense   = 0.11;
  const float VfsHigh = 0.32;
  _currentScale       = 32.f * sqrtf(2) * ampere * (Rsense + 0.02f) / VfsHigh - 1.f;

  if (_currentScale < 16) {
    const float VfsLow = 0.18;
    _currentScale      = 32.f * sqrtf(2) * ampere * (Rsense + 0.02f) / VfsLow - 1.f;

    // High sensitivity, low sense resistor voltage.
    tmc2130_writeField(&_tmc2130, TMC2130_VSENSE_FIELD, 1);
  }
}
