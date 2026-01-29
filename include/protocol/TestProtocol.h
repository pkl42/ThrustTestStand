/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TEST_PROTOCOL_H
#define TEST_PROTOCOL_H

#include <Arduino.h>
#include <vector>
#include "core/Config.h" // only for the default values of the struct ProtocolLimits

/**
 * @brief Supported test step types.
 *
 * Defines how motor throttle is controlled during a protocol step.
 */
enum class TestStepType : uint8_t
{
    Set = 0,      ///< Set throttle immediately and proceed
    Hold = 1,     ///< Hold throttle for a fixed duration
    Ramp = 2,     ///< Linearly ramp throttle over time
    StepSweep = 3 ///< Discrete stepped sweep over a range
};

/**
 * @brief Motion behavior configuration.
 *
 * Controls how throttle transitions are executed.
 */
struct MotionConfig
{
    bool smooth = true;          ///< Enable smooth transitions
    uint32_t accelTimeMs = 1000; ///< Acceleration duration
    uint32_t decelTimeMs = 1000; ///< Deceleration duration

    /**
     * @brief Settling delay before measurement.
     *
     * Delay after reaching the target throttle before sensor
     * data accumulation begins. The dwell time is counted
     * after this delay.
     */
    uint32_t settleTimeMs = 200;
};

/**
 * @brief Measurement completion mode.
 */
enum class MeasurementCompletion
{
    FixedDuration, ///< Stop measurement after a fixed time
    Periodic,      ///< Periodic time-series sampling
    OnStop         ///< Continue until the step stops
};

/**
 * @brief Measurement configuration.
 *
 * Controls if and how sensor data is recorded.
 */
struct MeasurementConfig
{
    bool enabled = true; ///< Enable measurement

    MeasurementCompletion completion = MeasurementCompletion::OnStop;

    uint32_t intervalS = 60; ///< Interval for periodic measurements (seconds)
    uint32_t windowMs = 20;  ///< Sampling window duration (ms)
};

/**
 * @brief Single protocol step definition.
 *
 * Describes a unit of motor control and measurement.
 */
struct TestStep
{
    TestStepType type; ///< Step execution type

    float throttleFrom = 0.0f; ///< Starting throttle (%)
    float throttleTo = 0.0f;   ///< Target throttle (%)

    float rampRatePctPerS = 1.f; ///< Ramp rate (% per second)
    float stepPct = 5.f;         ///< Discrete step increment (%)

    uint32_t dwellMs = 0; ///< Dwell time at target throttle (ms)

    /**
     * @brief Optional per-step motion overrides.
     *
     * Overrides header-level motion defaults if present.
     */
    MotionConfig motion;

    /**
     * @brief Optional per-step measurement overrides.
     *
     * Overrides header-level measurement defaults if present.
     */
    MeasurementConfig measure;
};

/**
 * @brief Safety and operational limits for protocol execution.
 *
 * Limits defined here must not weaken firmware-level safety limits.
 */
struct ProtocolLimits
{
    float maxCurrentA = CURRENT_SENSOR_MAX;  ///< Maximum current (A)
    float maxTempC = TEMPERATURE_SENSOR_MAX; ///< Maximum temperature (°C)
    uint32_t maxDurationS = 2592000;         ///< Maximum protocol duration (s)
    float maxVoltageV = VOLTAGE_SENSOR_MAX;  ///< Maximum voltage (V)
    float minVoltageV = 0.f;                 ///< Minimum voltage (V)
    float maxThrustGF = THRUST_SENSOR_MAX;   ///< Maximum thrust (gF)
};

/**
 * @brief Complete test protocol definition.
 *
 * A TestProtocol defines a sequence of steps along with default
 * motion, measurement, and safety settings.
 *
 * Configuration precedence:
 * - Firmware safety limits (strongest)
 * - Protocol header limits
 * - Per-step overrides (cannot weaken limits)
 */
struct TestProtocol
{
    char id[64];      ///< Protocol identifier
    char name[64];    ///< Human-readable protocol name
    char version[16]; ///< Protocol version string

    uint32_t estDurationS = 0; ///< Estimated total duration (s)

    /**
     * @brief Total execution units.
     *
     * Used for progress reporting. StepSweep steps may
     * contribute multiple units.
     */
    uint32_t unitsTotal = 0;

    ProtocolLimits limits; ///< Protocol safety limits

    MotionConfig motion; ///< Header-level motion defaults

    MeasurementConfig measurement; ///< Header-level measurement defaults

    std::vector<TestStep> steps; ///< Ordered list of protocol steps
};

#endif // TEST_PROTOCOL_H
