/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BASE_ACTUATOR_H
#define BASE_ACTUATOR_H

#include "ActuatorState.h"
#include "esp_log.h"

class BaseActuator
{
public:
    virtual ~BaseActuator() = default;

    virtual bool begin() = 0;
    virtual bool update() = 0;
    virtual void stop() = 0;          // normal stop → ACTU_STOPPED
    virtual void stopWithError() = 0; // → ACTU_ERROR

    ActuatorState getState() const { return _actuatorState; }

    bool isReady() const
    {
        return _actuatorState != ActuatorState::ACTU_UNINITIALIZED &&
               _actuatorState != ActuatorState::ACTU_INITIALIZING &&
               _actuatorState != ActuatorState::ACTU_ERROR &&
               _actuatorState != ActuatorState::ACTU_DISABLED &&
               _actuatorState != ActuatorState::ACTU_E_STOP;
    }

    bool canActuate() const
    {
        return _actuatorState == ActuatorState::ACTU_READY ||
               _actuatorState == ActuatorState::ACTU_ACTIVE ||
               _actuatorState == ActuatorState::ACTU_STOPPED;
    }

    bool isEmergencyStopped() const
    {
        return _actuatorState == ActuatorState::ACTU_E_STOP;
    }

    // -------- Safety hooks --------

    virtual void emergencyStop(const char *reason)
    {
        if (_actuatorState == ActuatorState::ACTU_E_STOP)
            return;

        ESP_LOGE(_tag, "EMERGENCY STOP: %s", reason);

        forceStopHardware();
        // Direct assignment intentional: E-STOP bypasses normal state gating
        _actuatorState = ActuatorState::ACTU_E_STOP;
    }

    virtual void resetEmergencyStop()
    {
        ESP_LOGW(_tag, "E-STOP reset requested");

        if (_actuatorState != ActuatorState::ACTU_E_STOP)
            return;

        _actuatorState = ActuatorState::ACTU_STOPPED;
    }

    virtual void resetError()
    {
        if (_actuatorState == ActuatorState::ACTU_ERROR)
        {
            ESP_LOGW(_tag, "Resetting error state, actuator now STOPPED");
            _actuatorState = ActuatorState::ACTU_STOPPED;
        }
    }

    virtual void disable()
    {
        stop();
        setState(ActuatorState::ACTU_DISABLED);
    }

protected:
    BaseActuator(const char *tag)
        : _tag(tag) {}

    virtual void forceStopHardware() = 0;

    void setState(ActuatorState newState)
    {
        if (_actuatorState == ActuatorState::ACTU_E_STOP)
            return; // nothing overrides E-STOP

        _actuatorState = newState;
    }

    const char *_tag;
    ActuatorState _actuatorState = ActuatorState::ACTU_UNINITIALIZED;
};

#endif
