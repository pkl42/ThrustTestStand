/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSOR_STATE_STRINGS_H
#define SENSOR_STATE_STRINGS_H

#include "SensorState.h"

inline const char *toString(SensorState s)
{
    switch (s)
    {
    case SensorState::SENSOR_UNINITIALIZED:
        return "UNINITIALIZED";
    case SensorState::SENSOR_INITIALIZING:
        return "INITIALIZING";
    case SensorState::SENSOR_READY:
        return "READY";
    case SensorState::SENSOR_ERROR:
        return "ERROR";
    case SensorState::SENSOR_DISABLED:
        return "DISABLED";
    default:
        return "UNKNOWN";
    }
}

#endif // SENSOR_STATE_STRINGS_H
