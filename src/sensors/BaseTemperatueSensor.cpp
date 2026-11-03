/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "sensors/BaseTemperatureSensor.h"
#include <esp_log.h>

void BaseTemperatureSensor::setCalibration(float coefficient, float offset)
{
    _calibration.coefficient = coefficient;
    _calibration.offset = offset;
}

BaseTemperatureSensor::Calibration BaseTemperatureSensor::getCalibration() const
{
    return _calibration;
}

float BaseTemperatureSensor::getTemperatureInCelsius() const
{
    return _temperature;
}