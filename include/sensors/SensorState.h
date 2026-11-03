/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H
#include <stdint.h>

/**
 * @enum SensorState
 * @brief Lifecycle and operational states for all sensors.
 *
 * Defines the standardized state model used by @ref BaseSensor and all
 * derived sensor implementations.
 *
 * @section sensor_states State Overview
 *
 * Typical lifecycle:
 *
 * @code
 * UNINITIALIZED → INITIALIZING → READY
 *                        ↓
 *                      ERROR
 * @endcode
 *
 * DISABLED may be entered from any state via user/system control.
 *
 *
 * @section sensor_state_semantics State Semantics
 *
 * - SENSOR_UNINITIALIZED: Default state before initialization
 * - SENSOR_INITIALIZING: Hardware setup or calibration in progress
 * - SENSOR_READY: Sensor initialized and operational
 * - SENSOR_ERROR: Fault detected, requires recovery or reset
 * - SENSOR_DISABLED: Sensor intentionally disabled, no updates performed
 *
 *
 * @section sensor_behavior Behavior Notes
 *
 * - SENSOR_READY does NOT guarantee valid data
 *   → see BaseSensor::hasValidData()
 *
 * - SENSOR_ERROR indicates a persistent failure condition
 *   (e.g. communication failure, hardware fault)
 *
 * - SENSOR_DISABLED prevents updates and data acquisition
 *
 *
 * @section sensor_usage Usage Guidelines
 *
 * - isInitialized() typically checks for READY or ERROR states
 * - isReady() typically requires READY and not DISABLED
 * - hasValidData() must be used in addition to state checks
 *
 *
 * @note
 * State transitions are controlled by derived classes via @ref BaseSensor.
 *
 * @see BaseSensor
 */
enum class SensorState : uint8_t
{
    SENSOR_UNINITIALIZED = 0,
    SENSOR_INITIALIZING,
    SENSOR_READY,
    SENSOR_ERROR,
    SENSOR_DISABLED
};

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

#endif // SENSOR_STATE_H
