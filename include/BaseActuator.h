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
    virtual void stop() = 0;

    ActuatorState getState() const { return _state; }

    bool isReady() const
    {
        return _state != ActuatorState::ACTU_UNINITIALIZED &&
               _state != ActuatorState::ACTU_INITIALIZING &&
               _state != ActuatorState::ACTU_ERROR &&
               _state != ActuatorState::ACTU_DISABLED &&
               _state != ActuatorState::ACTU_E_STOP;
    }

    bool canActuate() const
    {
        return _state == ActuatorState::ACTU_READY ||
               _state == ActuatorState::ACTU_ACTIVE ||
               _state == ActuatorState::ACTU_STOPPED;
    }

    bool isEmergencyStopped() const
    {
        return _state == ActuatorState::ACTU_E_STOP;
    }

    // -------- Safety hooks --------

    virtual void emergencyStop(const char *reason)
    {
        if (_state == ActuatorState::ACTU_E_STOP)
            return;

        ESP_LOGE(_tag, "EMERGENCY STOP: %s", reason);

        forceStopHardware();
        _state = ActuatorState::ACTU_E_STOP;
    }

    virtual void resetEmergencyStop()
    {
        ESP_LOGW(_tag, "E-STOP reset requested");

        if (_state != ActuatorState::ACTU_E_STOP)
            return;

        _state = ActuatorState::ACTU_STOPPED;
    }

    virtual void disable()
    {
        stop();
        _state = ActuatorState::ACTU_DISABLED;
    }

protected:
    BaseActuator(const char *tag)
        : _tag(tag) {}

    virtual void forceStopHardware() = 0;

    void setState(ActuatorState newState)
    {
        if (_state == ActuatorState::ACTU_E_STOP)
            return; // nothing overrides E-STOP

        _state = newState;
    }

    const char *_tag;
    ActuatorState _state = ActuatorState::ACTU_UNINITIALIZED;
};

#endif
