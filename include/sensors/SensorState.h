/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H
#include <stdint.h>

enum class SensorState : uint8_t
{
    SENSOR_UNINITIALIZED = 0,
    SENSOR_INITIALIZING,
    SENSOR_READY,
    SENSOR_ERROR,
    SENSOR_DISABLED
};

#endif // SENSOR_STATE_H
