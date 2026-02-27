/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "core/Config.h"

#if TEMPERATURE_SENSOR_TYPE == TEMP_SENSOR_MAX31855
#include "sensors/TempMAX31855Sensor.h"
using BaseTemperatureSensor = TempMax31855Sensor;

#elif TEMPERATURE_SENSOR_TYPE == TEMP_SENSOR_MLX90614
#include "sensors/TempMAX90614Sensor.h"
using BaseTemperatureSensor = TempMax90614Sensor;

#else
#include "BaseSensor.h"
class DummyTemperatureSensor : public BaseSensor
{
public:
    bool begin(...) { return false; }
    bool update() override { return false; }
    float getTemperatureInCelsius() const { return 0.0f; }
};
using BaseTemperatureSensor = DummyTemperatureSensor;

#endif

class TemperatureSensor : public BaseTemperatureSensor
{
public:
    bool begin()
    {
#if TEMPERATURE_SENSOR_TYPE == TEMP_SENSOR_MAX31855
        return BaseTemperatureSensor::begin(MAX31855_CLK_PIN, MAX31855_CS_PIN, MAX31855_SO_PIN);
#elif TEMPERATURE_SENSOR_TYPE == TEMP_SENSOR_MLX90614
        return BaseTemperatureSensor::begin(MLX90614_SDA_PIN, MLX90614_SCL_PIN);
#else
        return BaseTemperatureSensor::begin(); // dummy
#endif
    }
};

#endif // TEMPERATURE_SENSOR_H