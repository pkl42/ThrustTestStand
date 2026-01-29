/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "sensors/ThrustSensor.h"
#include <esp_log.h>

ThrustSensor::ThrustSensor(uint8_t doutPin, uint8_t sckPin)
    : _loadCell(doutPin, sckPin) // ✅ constructed here
{
}

bool ThrustSensor::begin(float calibration)
{
    if (isReady())
    {
        setCalibration(calibration);
        ESP_LOGI(TAG, "HX711 Thrust Loadcell already initialized and ready.");
        return true;
    }
    ESP_LOGI(TAG, "Initialize HX711 Thrust Loadcell ...");
    setState(SensorState::SENSOR_INITIALIZING);
    _loadCell.begin();
    _loadCell.setGain(128);

    unsigned long stabilizingTime = 3000;
    bool tare = true;
    while (!_loadCell.startMultiple(stabilizingTime, tare))
    {
    }

    if (_loadCell.getTareTimeoutFlag())
    {
        ESP_LOGE("ThrustSensor", "Timeout, check MCU => HX711 Thrust Loadcell wiring and pin designations");
        setState(SensorState::SENSOR_ERROR);
        incrementError();
        return false;
    }

    setCalibration(calibration);

    setDataValid(false);

    setState(SensorState::SENSOR_READY);
    ESP_LOGI(TAG, "HX711 Thrust Loadcell initialized.");
    return true;
}

void ThrustSensor::setCalibration(float calibration)
{
    if (calibration != 0.f) // 0.f is indicator for value not being set by caller
    {
        _loadCell.setCalFactor(calibration);
    }
}

bool ThrustSensor::update()
{
    if (!isReady())
    {
        ESP_LOGI(TAG, "HX711 Thrust Loadcell not ready.");
        return false;
    }

    if (_loadCell.update() == 0)
    {
        return false;
    }

    _thrust = _loadCell.getData();
    setDataValid(true);
    ++_updateCount;
    return true;
}

bool ThrustSensor::tare()
{
    if (!isReady())
        return false;
    ESP_LOGI("ThrustSensor", "Taring sensor ...");
    _loadCell.tareNoDelay();
    ESP_LOGI("ThrustSensor", "done");
    return true;
}
