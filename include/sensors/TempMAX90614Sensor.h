/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TEMP_MAX90614_SENSOR_H
#define TEMP_MAX90614_SENSOR_H

#include "BaseTemperatureSensor.h"
#include <Adafruit_MLX90614.h>
#include <Wire.h>

/**
 * @brief Infrared temperature sensor using MLX90614 (I2C)
 *
 * This class wraps the Adafruit_MLX90614 driver and integrates it into the
 * BaseTemperatureSensor/BaseSensor framework. It provides non-blocking temperature updates and
 * tracks data validity.
 */
class TempMax90614Sensor : public BaseTemperatureSensor
{
public:
    TempMax90614Sensor() = default;
    ~TempMax90614Sensor() = default;

    /**
     * @brief Initialize the MLX90614 sensor (I2C)
     *
     * Must be called before update().
     *
     * @param sdaPin I2C SDA pin
     * @param sclPin I2C SCL pin
     * @param frequency I2C clock frequency (default 100kHz)
     *
     * @return true if initialization succeeded
     */
    bool begin(uint8_t sdaPin, uint8_t sclPin, uint32_t frequency = 100000);

    /**
     * @brief Read and update the latest object temperature
     *
     * @return true if valid temperature was read
     */
    bool update() override;

private:
    const char *TAG = "MAX90614_Sensor";

    Adafruit_MLX90614 _mlx;
};

#endif // TEMP_MAX90614_SENSOR_H
