/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SAFETY_LIMITS_H
#define SAFETY_LIMITS_H

#include "core/Config.h"

/**
 * @enum BatteryPreset
 * @brief Predefined battery configurations based on LiPo cell count.
 *
 * Used to configure expected voltage ranges and simplify user selection.
 * The numeric value corresponds to the number of LiPo cells (S).
 *
 * @note
 * BATTERY_PRESET_NONE is typically used for calibration or when
 * no battery-based limits should be applied.
 */
enum class BatteryPreset : uint8_t
{
    BATTERY_PRESET_NONE = 0,
    BATTERY_PRESET_3S = 3,
    BATTERY_PRESET_4S = 4,
    BATTERY_PRESET_5S = 5,
    BATTERY_PRESET_6S = 6,
};

/**
 * @brief Convert battery preset to human-readable label.
 *
 * @param preset Battery preset
 * @return Constant string, never null.
 */
inline const char *batteryPresetToString(BatteryPreset preset)
{
    switch (preset)
    {
    case BatteryPreset::BATTERY_PRESET_NONE:
        return "None / Calibration";
    case BatteryPreset::BATTERY_PRESET_3S:
        return "3S LiPo";
    case BatteryPreset::BATTERY_PRESET_4S:
        return "4S LiPo";
    case BatteryPreset::BATTERY_PRESET_5S:
        return "5S LiPo";
    case BatteryPreset::BATTERY_PRESET_6S:
        return "6S LiPo";
    default:
        return "Unknown Battery";
    }
}

/**
 * @struct SafetyLimits
 * @brief Configurable safety limits for electrical and mechanical parameters.
 *
 * Defines upper and lower bounds for monitored values such as current,
 * voltage, thrust, and temperature. These limits are evaluated by the
 * safety system and may trigger a safety trip if exceeded.
 *
 * @section limits_behavior Limit Behavior
 * - Limits are interpreted as absolute thresholds
 * - A value ≤ 0 may disable the corresponding limit (implementation-dependent)
 * - Limits may be applied independently or in combination
 *
 * @section limits_scope Scope
 * - coreLimits: Always active (hardware protection)
 * - testLimits: Optional limits for experiment/test scenarios
 *
 * @note
 * This structure contains configuration only. It does not perform validation
 * or enforcement by itself.
 */
struct SafetyLimits
{
    float maxCurrentA = CURRENT_SENSOR_MAX;         ///< Maximum allowed motor current [A]
    float maxThrottlePercent = 100.f;               ///< Maximum allowed throttle [%]
    float maxVoltageV = VOLTAGE_SENSOR_MAX;         ///< Maximum allowed supply voltage [V]
    float minVoltageV = 0.f;                        ///< Minimum allowed supply voltage [V]
    float maxThrustGF = THRUST_SENSOR_MAX;          ///< Maximum allowed thrust [gf]
    float maxTemperatureC = TEMPERATURE_SENSOR_MAX; ///< Maximum allowed Motor Temperature [°C]
    BatteryPreset batteryPreset = BatteryPreset::BATTERY_PRESET_NONE;
};

/**
 * @brief Convert LiPo cell count to BatteryPreset.
 *
 * @param cells Number of cells (2–6 supported)
 * @return Corresponding preset or BATTERY_PRESET_NONE if invalid
 */
constexpr inline BatteryPreset cellsToPreset(uint8_t cells)
{
    return (cells >= 2 && cells <= 6)
               ? static_cast<BatteryPreset>(cells)
               : BatteryPreset::BATTERY_PRESET_NONE;
}

/**
 * @brief Convert BatteryPreset to cell count.
 *
 * @param preset Battery preset
 * @return Number of cells (0 if NONE)
 */
constexpr inline uint8_t batteryPresetToCells(BatteryPreset preset)
{
    return static_cast<uint8_t>(preset);
}

/**
 * @enum SafetyTripReason
 * @brief High-level causes of safety shutdown conditions.
 *
 * Represents the primary reason why the system entered a safety trip state.
 * Used for diagnostics, logging, and user feedback.
 *
 * @section trip_categories Categories
 *
 * Electrical limits:
 * - OVER_CURRENT
 * - OVER_VOLTAGE
 * - UNDER_VOLTAGE
 *
 * Mechanical limits:
 * - OVER_THRUST
 * - OVER_TEMPERATURE
 *
 * System faults:
 * - SENSOR_FAILURE
 * - ACTUATOR_FAILURE
 * - CONTROL_FAULT
 *
 * External / user events:
 * - USER_ABORT
 * - PROP_CAGE_OPEN
 * - ESTOP
 * - PASSTHROUGH
 *
 * Control violations:
 * - OVER_THROTTLE
 *
 * @note
 * Only the primary cause is stored. Additional diagnostic data should
 * be logged separately if needed.
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

    SAFETY_TRIP_ESTOP,       ///< EStop Button triggered
    SAFETY_TRIP_PASSTHROUGH, ///< Entering Passthrough Mode, similiar to pressing EStop Button

    SAFETY_TRIP_OVER_THROTTLE ///< requested throttle exceeded configured limit
};

/**
 * @brief Convert SafetyTripReason to human-readable string.
 *
 * @param reason Trip reason
 * @return Constant string, never null.
 */
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
    case SafetyTripReason::SAFETY_TRIP_PASSTHROUGH:
        return "Passthrough Mode";
    case SafetyTripReason::SAFETY_TRIP_OVER_THROTTLE:
        return "Over Throttle";
    default:
        return "Unknown";
    }
}

/**
 * @enum SafetyTripSource
 * @brief Origin of the applied safety limit.
 *
 * Distinguishes whether a trip was caused by:
 * - Core safety limits (hardware protection)
 * - Test-specific limits (user-defined constraints)
 */
enum class SafetyTripSource
{
    CORE_LIMIT,
    TEST_LIMIT,
    NONE
};

/**
 * @brief Convert SafetyTripSource to human-readable string.
 */
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
 * @struct SafetyState
 * @brief Current safety supervision status.
 *
 * Represents whether a safety trip has occurred and provides
 * diagnostic information about the event.
 *
 * @section behavior Behavior
 * - tripped == true indicates system is in a faulted state
 * - reason identifies the primary cause
 * - tripValue stores the measured value at violation
 *
 * @note
 * Values are only meaningful when tripped == true.
 */
struct SafetyState
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
};

/**
 * @struct ThrustStandSafety
 * @brief Aggregated safety configuration and runtime state.
 *
 * Combines:
 * - Core safety limits (always enforced)
 * - Test-specific limits (optional constraints)
 * - Current safety state
 *
 * @section usage Usage
 * - coreLimits protect hardware
 * - testLimits restrict experiments
 * - state reflects current system safety condition
 *
 * @note
 * Evaluation logic is implemented elsewhere (not in this structure).
 */
struct ThrustStandSafety
{
    SafetyLimits coreLimits; ///< Configured core safety limits
    SafetyLimits testLimits; ///< Configured test safety limits
    SafetyState state;       ///< Current safety status
};

#endif // SAFETY_LIMITS_H
