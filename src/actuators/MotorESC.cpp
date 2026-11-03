/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <memory>
#include "actuators/MotorESC.h"
#include <esp_log.h>
#include "driver/ledc.h"

static const char *TAG = "MotorESC";

MotorESC::MotorESC(uint8_t escPin)
    : BaseActuator(TAG), _driver(nullptr)
{
    _escPin = escPin;
    _mountState = MountState::DISARMED;
    _currentThrottle = 0.0f;
    _targetThrottle = 0.0f;
    _smoothingActive = false;
}

bool MotorESC::begin()
{
    setState(ActuatorState::ACTU_INITIALIZING);
    _mountState = MountState::DISARMED;

    if (!_driver)
    {
        setState(ActuatorState::ACTU_ERROR);
        ESP_LOGE(TAG, "ESC driver not constructed (null)");
        return false;
    }

    bool retState = disarm(true);
    setState(ActuatorState::ACTU_STOPPED);

    ESP_LOGI(TAG, "ESC ready using driver %p", _driver.get());

    return true;
}

bool MotorESC::arm(uint16_t armTimeMs)
{
    if (!isReady() || _mountState != MountState::DISARMED)
        return false;

    ESP_LOGI(TAG, "Starting ESC arming sequence (%u ms)", armTimeMs);

    if (!_driver->init())
    {
        setState(ActuatorState::ACTU_ERROR);
        ESP_LOGE(TAG, "ESC driver initialization failed");
        return false;
    }

    _driver->writeIdle(); // hold min throttle
    _armStartTime = millis();
    _armDuration = armTimeMs;
    _mountState = MountState::ARMING;

    return true;
}

bool MotorESC::disarm(bool forceFlag)
{
    if ((!isReady() || _mountState != MountState::ARMED) && !forceFlag)
        return false;

    if (_driver)
    {
        _driver->writeIdle();
    }
    resetThrottleValues();
    _mountState = MountState::DISARMED;
    return true;
}

void MotorESC::resetThrottleValues()
{
    _currentThrottle = 0.0f;
    _targetThrottle = 0.0f;
    _smoothingActive = false;
}

bool MotorESC::update()
{
    if (!isReady())
        return false;

    unsigned long now = millis();

    // ---------- ARMING ----------
    if (_mountState == MountState::ARMING)
    {
        if (now - _armStartTime >= _armDuration)
        {
            _mountState = MountState::ARMED;
            ESP_LOGI(TAG, "ESC armed");
        }
        return true;
    }

    // ---------- SMOOTHING ----------
    if (!_smoothingActive || _mountState == MountState::DISARMED)
        return false;

    float progress = (float)(now - _stateStartTime) / _transitionTimeMs;

    if (progress >= 1.0f)
    {
        _smoothingActive = false;
        updateThrottle(_targetThrottle);

        if (_targetThrottle <= 0.0f)
        {
            _mountState = MountState::ARMED;
            setState(ActuatorState::ACTU_STOPPED);
        }
        else
        {
            _mountState = MountState::RUNNING;
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
    if (!canActuate() || (_mountState != MountState::ARMED && _mountState != MountState::RUNNING))
        return 0.0f;

    if (smooth)
    {
        _startThrottle = _currentThrottle;
        _targetThrottle = throttlePercent;
        _transitionTimeMs = transitionTimeMs;
        _stateStartTime = millis();
        _smoothingActive = true;

        if (throttlePercent > 0.)
            _mountState = MountState::RUNNING;

        setState(ActuatorState::ACTU_ACTIVE);
        return _targetThrottle;
    }

    updateThrottle(throttlePercent);

    _targetThrottle = throttlePercent;
    _mountState = (throttlePercent > 0.f) ? MountState::RUNNING : MountState::ARMED;

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
    bool retState = disarm(true);
    setState(ActuatorState::ACTU_STOPPED);
}

void MotorESC::stopWithError()
{
    bool retState = disarm(true);
    setState(ActuatorState::ACTU_ERROR);
}

void MotorESC::forceStopHardware()
{
    bool retState = disarm(true);
}

bool MotorESC::isAtTargetSpeed(float tolerancePercent) const
{
    return fabs(_currentThrottle - _targetThrottle) <= tolerancePercent;
}

bool MotorESC::setDriverConfig(ESCConfig &driverConfig)
{
    if (!_driver)
        return false;
    if (getState() == ActuatorState::ACTU_ACTIVE)
        return false;

    _driverConfig = driverConfig;
    return _driver->setConfig(_driverConfig);
}

bool MotorESC::setDriver(EscDriverType type)
{
    if (_escDriverType == type)
        return true;

    if (_mountState != MountState::DISARMED)
        return false;

    stop(); // ensure ESC is not actively driving

    vTaskDelay(1);

    // --- Deinit old driver ---
    if (_driver)
    {
        _driver->writeIdle(); // safe state first
        _driver->deinit();
        _driver.reset();
    }

    // --- Handle NONE ---
    if (type == EscDriverType::ESC_DRIVER_NONE)
    {
        _escDriverType = type;
        return true;
    }

    // --- Create new driver ---
    std::unique_ptr<EscSignalDriver> driver;

    switch (type)
    {
    case EscDriverType::ESC_DRIVER_PWM:
        driver = std::unique_ptr<EscSignalDriver>(new PwmEscDriver(_escPin, 50, 13, 1));
        break;

    case EscDriverType::ESC_DRIVER_DSHOT300:
    case EscDriverType::ESC_DRIVER_DSHOT600:
    case EscDriverType::ESC_DRIVER_DSHOT1200:
    {
        DShotEscDriver::DShotRate rate;

        switch (type)
        {
        case EscDriverType::ESC_DRIVER_DSHOT600:
            rate = DShotEscDriver::DShotRate::DSHOT600;
            break;
        case EscDriverType::ESC_DRIVER_DSHOT1200:
            rate = DShotEscDriver::DShotRate::DSHOT1200;
            break;
        default:
            rate = DShotEscDriver::DShotRate::DSHOT300;
            break;
        }

        driver = std::unique_ptr<EscSignalDriver>(new DShotEscDriver(_escPin, rate));
        // needSignalRefreshTask = true;
        break;
    }

    default:

        return false;
    }

    if (!driver)
    {
        return false;
    }

    // --- Activate new driver ---
    _driver = std::move(driver);

    if (!_driver->init())
    {
        _driver.reset();
        return false;
    }
    _driver->setConfig(_driverConfig);

    _driver->writeIdle();
    _escDriverType = type;

    return true;
}
