/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "sensors/BaseSensor.h"

BaseSensor::BaseSensor()
    : _state(SensorState::SENSOR_UNINITIALIZED),
      _dataValid(false),
      _updateCount(0),
      _errorCount(0)
{
}

SensorState BaseSensor::getState() const
{
    return _state;
}

bool BaseSensor::isReady() const
{
    return _state == SensorState::SENSOR_READY;
}

bool BaseSensor::isInitialized() const
{
    return _state != SensorState::SENSOR_UNINITIALIZED;
}

bool BaseSensor::hasValidData() const
{
    return _dataValid;
}

uint32_t BaseSensor::getUpdateCount() const
{
    return _updateCount;
}

uint32_t BaseSensor::getErrorCount() const
{
    return _errorCount;
}

void BaseSensor::setState(SensorState state)
{
    _state = state;
}

void BaseSensor::setDataValid(bool valid)
{
    _dataValid = valid;
}

void BaseSensor::incrementError()
{
    ++_errorCount;
}
