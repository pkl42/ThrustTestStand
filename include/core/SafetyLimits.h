/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "core/Config.h"

#ifndef SAFETY_LIMITS_H
#define SAFETY_LIMITS_H

/**
 * @brief Aggregated safety and operational limits of the thrust stand.
 *
 * Defines absolute upper bounds for critical electrical and mechanical
 * parameters. These limits are enforced by the thrust stand controller
 * to protect hardware and ensure safe operation.
 *
 * A limit value of zero or less may be interpreted as "disabled"
 * depending on the specific parameter and implementation.
 *
 * This structure contains configuration data only and does not
 * represent measured state or control logic.
 */
typedef struct
{
    float maxCurrentA = CURRENT_SENSOR_MAX;         ///< Maximum allowed motor current [A]
    float maxThrottlePercent = 100.f;               ///< Maximum allowed throttle [%]
    float maxVoltageV = VOLTAGE_SENSOR_MAX;         ///< Maximum allowed supply voltage [V]
    float minVoltageV = -1.f;                       ///< Minimum allowed supply voltage [V]
    float maxThrustGF = THRUST_SENSOR_MAX;          ///< Maximum allowed thrust [gf]
    float maxTemperatureC = TEMPERATURE_SENSOR_MAX; ///< Maximum allowed Motor Temperature [°C]
} SafetyLimits;

/**
 * @brief Identifies the primary cause of a safety trip condition.
 *
 * Enumerates all protection and supervision conditions that may
 * trigger a safety-related shutdown of the thrust stand.
 *
 * The listed reasons represent high-level fault categories and are
 * intended for diagnostics, logging, and user feedback. Detailed
 * sensor data should be captured separately if required.
 */
enum class SafetyTripReason : uint8_t
{
    SAFETY_TRIP_NONE = 0, ///< No active safety trip

    SAFETY_TRIP_OVER_CURRENT,     ///< Motor current exceeded configured limit
    SAFETY_TRIP_OVER_VOLTAGE,     ///< Supply voltage exceeded configured limit
    SAFETY_TRIP_UNDER_VOLTAGE,    ///< Supply voltage below configured limit
    SAFETY_TRIP_OVER_THRUST,      ///< Measured thrust exceeded configured limit
    SAFETY_TRIP_OVER_TEMPERATURE, ///< Temperature exceeded configured limit,

    SAFETY_TRIP_SENSOR_FAILURE,   ///< Invalid or implausible sensor data, TO BE IMPLEMENTED
    SAFETY_TRIP_ACTUATOR_FAILURE, ///< Motor, ESC, or actuator failure, TO BE IMPLEMENTED
    SAFETY_TRIP_CONTROL_FAULT,    ///< Internal control or state machine error, TO BE IMPLEMENTED

    SAFETY_TRIP_USER_ABORT,     ///< User-initiated abort, TO BE IMPLEMENTED
    SAFETY_TRIP_PROP_CAGE_OPEN, ///< External interlock triggered, e.g propeller cage open

    SAFETY_TRIP_ESTOP, ///< EStop Button triggered

    SAFETY_TRIP_OVER_THROTTLE ///< requested throttle exceeded configured limit
};

inline const char *safetyTripReasonToString(SafetyTripReason reason)
{
    switch (reason)
    {
    case SafetyTripReason::SAFETY_TRIP_NONE:
        return "None";
    case SafetyTripReason::SAFETY_TRIP_OVER_CURRENT:
        return "Over Current";
    case SafetyTripReason::SAFETY_TRIP_OVER_VOLTAGE:
        return "Over Voltage";
    case SafetyTripReason::SAFETY_TRIP_UNDER_VOLTAGE:
        return "Under Voltage";
    case SafetyTripReason::SAFETY_TRIP_OVER_THRUST:
        return "Over Thrust";
    case SafetyTripReason::SAFETY_TRIP_OVER_TEMPERATURE:
        return "Over Temperature";
    case SafetyTripReason::SAFETY_TRIP_SENSOR_FAILURE:
        return "Sensor Failure";
    case SafetyTripReason::SAFETY_TRIP_ACTUATOR_FAILURE:
        return "Actuator Failure";
    case SafetyTripReason::SAFETY_TRIP_CONTROL_FAULT:
        return "Control Fault";
    case SafetyTripReason::SAFETY_TRIP_USER_ABORT:
        return "User Abort";
    case SafetyTripReason::SAFETY_TRIP_PROP_CAGE_OPEN:
        return "Prop Cage Open";
    case SafetyTripReason::SAFETY_TRIP_ESTOP:
        return "E-Stop";
    case SafetyTripReason::SAFETY_TRIP_OVER_THROTTLE:
        return "Over Throttle";
    default:
        return "Unknown";
    }
}

enum class SafetyTripSource
{
    CORE_LIMIT,
    TEST_LIMIT,
    NONE
};

inline const char *safetyTripSourceToString(SafetyTripSource source)
{
    switch (source)
    {
    case SafetyTripSource::CORE_LIMIT:
        return "Core Limit";
    case SafetyTripSource::TEST_LIMIT:
        return "Test Limit";
    case SafetyTripSource::NONE:
        return "None";
    default:
        return "Unknown";
    }
}

/**
 * @brief Aggregated safety status of the thrust stand.
 *
 * Represents the current safety supervision status of the system.
 * A safety trip indicates that an operational limit or protection
 * condition has been violated.
 */
typedef struct
{
    bool tripped = false; ///< Indicates an active safety trip condition

    /**
     * @brief Measured value at the time of the safety trip.
     *
     * Stores the sensor value that caused the safety trip
     * (e.g. current, voltage, thrust), depending on the
     * reported @ref reason. Valid only when @ref tripped is true.
     */
    float tripValue = 0.0f;

    /**
     * @brief Primary cause of the safety trip.
     *
     * Identifies the protection condition that caused the system
     * to enter a safety trip state. Valid only when @ref tripped
     * is true.
     */
    SafetyTripReason reason = SafetyTripReason::SAFETY_TRIP_NONE;
    SafetyTripSource source = SafetyTripSource::NONE;

} SafetyState;

struct ThrustStandSafety
{
    SafetyLimits coreLimits; ///< Configured core safety limits
    SafetyLimits testLimits; ///< Configured test safety limits
    SafetyState state;       ///< Current safety status
};

#endif // SAFETY_LIMITS_H
