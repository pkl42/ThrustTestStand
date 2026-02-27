/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "safety/HardwareEstop.h"
#include <esp_log.h>

static const char *TAG = "HW_ESTOP";

HardwareEstop::HardwareEstop(uint8_t gpioPin, bool activeLow)
    : _pin(gpioPin),
      _activeLow(activeLow)
{
}

void HardwareEstop::begin()
{
    pinMode(_pin, _activeLow ? INPUT_PULLDOWN : INPUT_PULLUP);

    bool level = digitalRead(_pin);
    _triggered = _activeLow ? (level == LOW) : (level == HIGH);

    attachInterruptArg(
        digitalPinToInterrupt(_pin),
        &HardwareEstop::isrHandler,
        this,
        CHANGE // reacts to press + wire break
    );

    ESP_LOGI(TAG, "Hardware E-STOP initialized on GPIO %d", _pin);
}

void HardwareEstop::attachActuator(BaseActuator *actuator)
{
    if (actuator)
        _actuators.push_back(actuator);
}

bool HardwareEstop::isTriggered() const
{
    return _triggered;
}

bool HardwareEstop::isPhysicallyActive() const
{
    bool level = digitalRead(_pin);
    return _activeLow ? (level == LOW) : (level == HIGH);
}

bool HardwareEstop::reset()
{
    if (millis() - _releaseTimestamp < 200)
        return false; // require 200ms stable release

    // Only allow reset if physical switch is released
    if (isPhysicallyActive())
        return false;

    _triggered = false;
    return true;
}

void IRAM_ATTR HardwareEstop::isrHandler(void *arg)
{
    static_cast<HardwareEstop *>(arg)->handleInterrupt();
}

void IRAM_ATTR HardwareEstop::handleInterrupt()
{
    bool level = digitalRead(_pin);
    bool active = _activeLow ? (level == LOW) : (level == HIGH);

    if (!active) // released edge
        _releaseTimestamp = millis();

    if (active)
        _triggered = true;
}
