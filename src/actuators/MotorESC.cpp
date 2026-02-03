/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "actuators/MotorESC.h"
#include <esp_log.h>
#include "driver/ledc.h"
#include <esp_log.h>

static const char *TAG = "MotorESC";

MotorESC::MotorESC(EscSignalDriver *driver)
    : BaseActuator(TAG), _driver(driver)
{
    _escState = EscState::DISARMED;
    _currentThrottle = 0.0f;
    _targetThrottle = 0.0f;
    _smoothingActive = false;
}

bool MotorESC::begin()
{
    setState(ActuatorState::ACTU_INITIALIZING);
    _escState = EscState::DISARMED;

    if (!_driver)
    {
        setState(ActuatorState::ACTU_ERROR);
        ESP_LOGE(TAG, "ESC driver not constructed (null)");
        return false;
    }

    if (!_driver->begin())
    {
        setState(ActuatorState::ACTU_ERROR);
        ESP_LOGE(TAG, "ESC driver initialization failed");
        return false;
    }

    forceStopHardware();
    setState(ActuatorState::ACTU_STOPPED);

    ESP_LOGI(TAG, "ESC ready using driver %p", _driver);
    return true;
}

bool MotorESC::arm(uint16_t armTimeMs)
{
    if (!isReady() || _escState != EscState::DISARMED)
        return false;

    ESP_LOGI(TAG, "Starting ESC arming sequence (%u ms)", armTimeMs);

    _driver->writeStop(); // hold min throttle
    _armStartTime = millis();
    _armDuration = armTimeMs;
    _escState = EscState::ARMING;

    return true;
}

bool MotorESC::disarm()
{
    if (!isReady() || _escState != EscState::ARMED)
        return false;

    forceStopHardware();
    return true;
}

bool MotorESC::update()
{
    if (!isReady())
        return false;

    unsigned long now = millis();

    // ---------- ARMING ----------
    if (_escState == EscState::ARMING)
    {
        if (now - _armStartTime >= _armDuration)
        {
            _escState = EscState::ARMED;
            ESP_LOGI(TAG, "ESC armed");
        }
        return true;
    }

    // ---------- SMOOTHING ----------
    if (!_smoothingActive || _escState == EscState::DISARMED)
        return false;

    float progress = (float)(now - _stateStartTime) / _transitionTimeMs;

    if (progress >= 1.0f)
    {
        _smoothingActive = false;
        updateThrottle(_targetThrottle);

        if (_targetThrottle <= 0.0f)
        {
            _escState = EscState::ARMED;
            setState(ActuatorState::ACTU_STOPPED);
        }
        else
        {
            _escState = EscState::RUNNING;
            setState(ActuatorState::ACTU_ACTIVE);
        }

        return true;
    }

    float throttle = _startThrottle + (_targetThrottle - _startThrottle) * progress;
    updateThrottle(throttle);

    return true;
}

float MotorESC::setThrottle(float throttlePercent, bool smooth, unsigned long transitionTimeMs)
{
    if (!canActuate() || (_escState != EscState::ARMED && _escState != EscState::RUNNING))
        return 0.0f;

    if (smooth)
    {
        _startThrottle = _currentThrottle;
        _targetThrottle = throttlePercent;
        _transitionTimeMs = transitionTimeMs;
        _stateStartTime = millis();
        _smoothingActive = true;

        if (throttlePercent > 0)
            _escState = EscState::RUNNING;

        setState(ActuatorState::ACTU_ACTIVE);
        return _targetThrottle;
    }

    updateThrottle(throttlePercent);

    _targetThrottle = throttlePercent;
    _escState = (throttlePercent > 0.f) ? EscState::RUNNING : EscState::ARMED;

    setState(throttlePercent > 0.f ? ActuatorState::ACTU_ACTIVE : ActuatorState::ACTU_STOPPED);

    return throttlePercent;
}

void MotorESC::updateThrottle(float throttlePercent)
{
    if (_driver)
        _driver->writeThrottle(throttlePercent);
    _currentThrottle = throttlePercent;
}

void MotorESC::stop()
{
    forceStopHardware();
    _escState = EscState::DISARMED;
    setState(ActuatorState::ACTU_STOPPED);
}

void MotorESC::stopWithError()
{
    forceStopHardware();
    _escState = EscState::DISARMED;
    setState(ActuatorState::ACTU_ERROR);
}

void MotorESC::forceStopHardware()
{
    if (_driver)
        _driver->writeStop();

    _currentThrottle = 0.0f;
    _targetThrottle = 0.0f;
    _smoothingActive = false;
    _escState = EscState::DISARMED;
}

bool MotorESC::isAtTargetSpeed(float tolerancePercent) const
{
    return fabs(_currentThrottle - _targetThrottle) <= tolerancePercent;
}

bool MotorESC::setPulseRangeUs(uint16_t minUs, uint16_t maxUs)
{
    if (!_driver)
        return false;

    if (getState() == ActuatorState::ACTU_ACTIVE)
        return false;

    return _driver->setPulseRangeUs(minUs, maxUs);
}

bool MotorESC::setDriver(EscSignalDriver *driver)
{
    if (!driver)
        return false;

    // Stop the old driver if it exists
    if (_driver)
        _driver->stop();

    // Assign the new driver
    _driver = driver;

    // Reset ESC state and throttle
    _escState = EscState::DISARMED;
    _currentThrottle = 0.0f;
    _targetThrottle = 0.0f;
    _smoothingActive = false;

    // Ensure hardware output is clean
    _driver->writeStop();
    return true;
}

void MotorESC::updateEsc()
{
    if (!_driver)
        return;

    if (isReady())
        _driver->update();
}
