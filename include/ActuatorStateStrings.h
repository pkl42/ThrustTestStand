#ifndef ACTUATOR_STATE_STRINGS_H
#define ACTUATOR_STATE_STRINGS_H

#include "ActuatorState.h"

inline const char *toString(ActuatorState s)
{
    switch (s)
    {
    case ActuatorState::ACTU_UNINITIALIZED:
        return "UNINITIALIZED";
    case ActuatorState::ACTU_INITIALIZING:
        return "INITIALIZING";
    case ActuatorState::ACTU_READY:
        return "READY";
    case ActuatorState::ACTU_ACTIVE:
        return "ACTIVE";
    case ActuatorState::ACTU_STOPPED:
        return "STOPPED";
    case ActuatorState::ACTU_ERROR:
        return "ERROR";
    case ActuatorState::ACTU_DISABLED:
        return "DISABLED";
    case ActuatorState::ACTU_E_STOP:
        return "E_STOP";
    default:
        return "UNKNOWN";
    }
}

#endif // ACTUATOR_STATE_STRINGS_H
