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

    bool retState = disarm(true);
    setState(ActuatorState::ACTU_STOPPED);

    ESP_LOGI(TAG, "ESC ready using driver %p", _driver.get());

    _lastLogTime = 0;
    startESCTask();

    return true;
}

void MotorESC::startESCTask()
{
    if (_taskESCHandle != nullptr)
        return;

    xTaskCreatePinnedToCore(
        escTaskTrampoline,
        "escTask",
        2048,
        this, // ✅ pass instance!
        3,
        &_taskESCHandle,
        1);
}

void MotorESC::stopESCTask()
{
    if (_taskESCHandle)
    {
        vTaskDelete(_taskESCHandle);
        _taskESCHandle = nullptr;
    }
}

bool MotorESC::arm(uint16_t armTimeMs)
{
    if (!isReady() || _escState != EscState::DISARMED)
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
    _escState = EscState::ARMING;

    return true;
}

bool MotorESC::disarm(bool forceFlag)
{
    if ((!isReady() || _escState != EscState::ARMED) && !forceFlag)
        return false;

    if (_driver)
    {
        _driver->writeIdle();
    }
    resetThrottleValues();
    _escState = EscState::DISARMED;
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

        if (throttlePercent > 0.)
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

bool MotorESC::setPulseRangeUs(uint16_t minUs, uint16_t maxUs)
{
    if (!_driver)
        return false;

    if (getState() == ActuatorState::ACTU_ACTIVE)
        return false;

    return _driver->setPulseRangeUs(minUs, maxUs);
}

void MotorESC::updateEsc()
{
    uint32_t now = millis();

    // if (now - _lastLogTime >= 5000)
    // {
    //     _lastLogTime = now;

    //     ESP_LOGI(TAG,
    //              "updateEsc | driver=%p | isReady=%s",
    //              _driver.get(),
    //              isReady() ? "true" : "false");
    // }

    if (_driver && isReady())
    {
        _driver->update();
    }
}

void MotorESC::escTaskTrampoline(void *param)
{
    static_cast<MotorESC *>(param)->escTaskLoop(param);
}

void MotorESC::escTaskLoop(void *parameter)
{
    const int delayUs = 100;
    int counter = 0;

    while (true)
    {
        updateEsc();
        ets_delay_us(delayUs);

        if (++counter >= 50) // every ~5 ms
        {
            counter = 0;
            vTaskDelay(1); // yield to idle task
        }
    }
}

bool MotorESC::setDriver(EscDriverType type)
{
    if (_escDriverType == type)
        return true;

    if (_escState != EscState::DISARMED)
        return false;

    bool needSignalRefreshTask = false;

    stop(); // ensure ESC is not actively driving

    stopESCTask(); // deletes FreeRTOS task

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

    case EscDriverType::ESC_DRIVER_DSHOT150:
    case EscDriverType::ESC_DRIVER_DSHOT300:
    case EscDriverType::ESC_DRIVER_DSHOT600:
    {
        DShotEscDriver::DShotRate rate;

        switch (type)
        {
        case EscDriverType::ESC_DRIVER_DSHOT150:
            rate = DShotEscDriver::DShotRate::DSHOT150;
            break;
        case EscDriverType::ESC_DRIVER_DSHOT600:
            rate = DShotEscDriver::DShotRate::DSHOT600;
            break;
        default:
            rate = DShotEscDriver::DShotRate::DSHOT300;
            break;
        }

        driver = std::unique_ptr<EscSignalDriver>(new DShotEscDriver(_escPin, rate));
        needSignalRefreshTask = true;
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

    // if (!begin())
    // {
    //     _driver.reset();
    //     resumeTask();
    //     return false;
    // }

    _driver->writeIdle();
    _escDriverType = type;

    if (needSignalRefreshTask)
    {
        startESCTask();
    }

    return true;
}
