/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sensors/TempMAX90614Sensor.h"
#include <Arduino.h>
#include <esp_log.h>

bool TempMax90614Sensor::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t frequency)
{
    if (isReady())
    {
        ESP_LOGI(TAG, "MAX90614 Infrared already initialized and ready.");
        return true;
    }

    ESP_LOGI(TAG, "Initialize MAX90614 Infrared ...");
    setState(SensorState::SENSOR_INITIALIZING);

    // Initialize I2C
    bool success = Wire.begin(sdaPin, sclPin, frequency);
    delay(50);

    ESP_LOGI(TAG, "Wire.begin %s ", success ? "successfully" : "NOT");

    // I2C Scanner for debugging
    ESP_LOGI(TAG, "I2C Scanning...");
    for (uint8_t addr = 0; addr < 128; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            ESP_LOGI(TAG, "Found device at 0x");
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }

    if (!_mlx.begin(MLX90614_I2CADDR, &Wire))
    {
        setState(SensorState::SENSOR_ERROR);
        setDataValid(false);
        // incrementError();
        ESP_LOGE(TAG, "Failed to initialize MAX90614 at init");
        return false;
    }

    setState(SensorState::SENSOR_READY);
    setDataValid(false);

    return true;
}

bool TempMax90614Sensor::update()
{
    if (_state != SensorState::SENSOR_READY)
        return false;

    float temp = _mlx.readObjectTempC();

    // MLX returns very low values on failure (often -273.15)
    if (isnan(temp) || temp < -100.0f || temp > 1000.0f)
    {
        setDataValid(false);
        incrementError();
        return false;
    }

    _temperature = temp;
    setDataValid(true);
    _updateCount++;

    return true;
}

float TempMax90614Sensor::getTemperatureInCelsius() const
{
    return _temperature;
}