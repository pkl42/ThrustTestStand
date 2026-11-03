/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BASE_ACTUATOR_H
#define BASE_ACTUATOR_H

#include "ActuatorState.h"
#include "esp_log.h"

/**
 * @class BaseActuator
 * @brief Base class for all actuators with unified state and safety handling.
 *
 * Provides:
 * - Standardized actuator lifecycle (begin, update, stop)
 * - Global state management via @ref ActuatorState
 * - Emergency stop (E-STOP) handling
 * - Error and disable handling
 *
 * Derived classes implement hardware-specific behavior.
 *
 * @section actuator_model Actuator State Model
 *
 * The actuator state is managed centrally and enforces safety rules:
 *
 * - ACTU_UNINITIALIZED → initial state
 * - ACTU_INITIALIZING  → during begin()
 * - ACTU_READY         → initialized, idle
 * - ACTU_ACTIVE        → actively actuating
 * - ACTU_STOPPED       → stopped but reusable
 * - ACTU_ERROR         → fault condition
 * - ACTU_DISABLED      → manually disabled
 * - ACTU_E_STOP        → emergency stop (highest priority)
 *
 * @section actuator_safety Safety Guarantees
 *
 * - E-STOP overrides all states and cannot be bypassed
 * - forceStopHardware() is always executed on E-STOP
 * - No state transitions are allowed once in ACTU_E_STOP
 *   except via resetEmergencyStop()
 *
 * @section actuator_responsibilities Responsibilities of Derived Classes
 *
 * Derived classes must:
 * - Implement begin(), update(), stop(), stopWithError()
 * - Implement forceStopHardware() to immediately disable hardware output
 * - Use setState() for all normal state transitions
 *
 * Derived classes must NOT:
 * - Override E-STOP behavior
 * - Bypass setState() (except where explicitly allowed)
 *
 * @note
 * This class does not enforce timing, control logic, or feedback validation.
 * These are handled at higher levels (e.g. MotorESC).
 */
class BaseActuator
{
public:
    virtual ~BaseActuator() = default;
    /**
     * @brief Initialize the actuator.
     * @return true if initialization succeeded
     *
     * Expected to transition state to ACTU_READY on success.
     */
    virtual bool begin() = 0;

    /**
     * @brief Periodic update function.
     * @return true if update succeeded
     *
     * Called regularly to process actuator logic.
     */
    virtual bool update() = 0;

    /**
     * @brief Perform a normal stop.
     *
     * Expected to transition to ACTU_STOPPED.
     */
    virtual void stop() = 0;

    /**
     * @brief Stop due to error condition.
     *
     * Expected to transition to ACTU_ERROR.
     */
    virtual void stopWithError() = 0;

    /**
     * @brief Get current actuator state.
     */
    ActuatorState getState() const { return _actuatorState; }

    /**
     * @brief Check if actuator is initialized and usable.
     *
     * @return true if not in UNINITIALIZED, INITIALIZING, ERROR,
     * DISABLED, or E-STOP.
     */
    bool isReady() const
    {
        return _actuatorState != ActuatorState::ACTU_UNINITIALIZED &&
               _actuatorState != ActuatorState::ACTU_INITIALIZING &&
               _actuatorState != ActuatorState::ACTU_ERROR &&
               _actuatorState != ActuatorState::ACTU_DISABLED &&
               _actuatorState != ActuatorState::ACTU_E_STOP;
    }

    /**
     * @brief Check if actuator is allowed to produce output.
     *
     * @return true if in READY, ACTIVE, or STOPPED state.
     */
    bool canActuate() const
    {
        return _actuatorState == ActuatorState::ACTU_READY ||
               _actuatorState == ActuatorState::ACTU_ACTIVE ||
               _actuatorState == ActuatorState::ACTU_STOPPED;
    }

    /**
     * @brief Check if actuator is in emergency stop state.
     */
    bool isEmergencyStopped() const
    {
        return _actuatorState == ActuatorState::ACTU_E_STOP;
    }

    // -------- Safety hooks --------

    /**
     * @brief Trigger emergency stop (E-STOP).
     *
     * Immediately stops hardware and forces ACTU_E_STOP state.
     *
     * @param reason Optional debug message
     *
     * @note
     * - This operation bypasses normal state transitions.
     * - Once in ACTU_E_STOP, no further state changes are allowed.
     */
    virtual void emergencyStop(const char *reason)
    {
        if (_actuatorState == ActuatorState::ACTU_E_STOP)
            return;

        ESP_LOGE(_tag, "EMERGENCY STOP: %s", reason);

        forceStopHardware();
        // Direct assignment intentional: E-STOP bypasses normal state gating
        _actuatorState = ActuatorState::ACTU_E_STOP;
    }

    /**
     * @brief Trigger emergency stop (E-STOP).
     *
     * Immediately stops hardware and forces ACTU_E_STOP state.
     *
     * @param reason Optional debug message
     *
     * @note
     * - This operation bypasses normal state transitions.
     * - Once in ACTU_E_STOP, no further state changes are allowed.
     */
    virtual void resetEmergencyStop()
    {
        ESP_LOGW(_tag, "E-STOP reset requested");

        if (_actuatorState != ActuatorState::ACTU_E_STOP)
            return;

        _actuatorState = ActuatorState::ACTU_STOPPED;
    }

    // -------- Error Handling --------

    /**
     * @brief Reset error state.
     *
     * Transitions ACTU_ERROR → ACTU_STOPPED.
     */
    virtual void resetError()
    {
        if (_actuatorState == ActuatorState::ACTU_ERROR)
        {
            ESP_LOGW(_tag, "Resetting error state, actuator now STOPPED");
            _actuatorState = ActuatorState::ACTU_STOPPED;
        }
    }

    /**
     * @brief Disable actuator.
     *
     * Stops actuator and prevents further actuation.
     *
     * Transitions to ACTU_DISABLED.
     */
    virtual void disable()
    {
        stop();
        setState(ActuatorState::ACTU_DISABLED);
    }

protected:
    BaseActuator(const char *tag)
        : _tag(tag) {}

    /**
     * @brief Immediately stop hardware output.
     *
     * Must be implemented by derived classes.
     *
     * @note
     * - Must be safe to call in emergency situations.
     * - Should not rely on dynamic allocation or blocking operations.
     */
    virtual void forceStopHardware() = 0;

    /**
     * @brief Set actuator state.
     *
     * @param newState Desired state
     *
     * @note
     * - Ignored if current state is ACTU_E_STOP.
     */
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
