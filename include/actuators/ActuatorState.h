/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACTUATOR_STATE_H
#define ACTUATOR_STATE_H

#include <stdint.h>

/**
 * @enum ActuatorState
 * @brief Unified lifecycle and safety states for all actuators.
 *
 * Defines the global state machine used by @ref BaseActuator and all derived
 * actuator implementations (e.g. MotorESC).
 *
 * @section actuator_states State Overview
 *
 * Typical lifecycle:
 *
 * @code
 * UNINITIALIZED → INITIALIZING → READY → ACTIVE
 *                        ↓           ↓
 *                    STOPPED ←───────┘
 *
 * ERROR   → requires resetError()
 * DISABLED → requires manual enable
 * E_STOP  → requires resetEmergencyStop()
 * @endcode
 *
 * @section actuator_state_semantics State Semantics
 *
 * - UNINITIALIZED: Default state before begin()
 * - INITIALIZING: Hardware setup in progress
 * - READY: Initialized, idle, allowed to actuate
 * - ACTIVE: Actuator is actively producing output
 * - STOPPED: Stopped but healthy and reusable
 * - ERROR: Fault detected, requires reset
 * - DISABLED: Software-disabled, no actuation allowed
 * - E_STOP: Emergency stop, latched until explicitly reset
 *
 * @section actuator_safety Safety Rules
 *
 * - ACTU_E_STOP has highest priority and overrides all other states
 * - No transitions are allowed out of ACTU_E_STOP except via resetEmergencyStop()
 * - ACTU_ERROR requires explicit reset before reuse
 * - ACTU_DISABLED prevents any actuation regardless of commands
 *
 * @section actuator_usage Usage Notes
 *
 * - isReady() typically checks for READY/ACTIVE/STOPPED states
 * - canActuate() is true only in READY, ACTIVE, STOPPED
 * - State transitions are managed by @ref BaseActuator
 *
 * @see BaseActuator
 */
enum class ActuatorState : uint8_t
{
    ACTU_UNINITIALIZED = 0, ///< Not initialized, begin() not yet called
    ACTU_INITIALIZING,      ///< Hardware initialization in progress
    ACTU_READY,             ///< Initialized and idle, ready for operation
    ACTU_ACTIVE,            ///< Actuator currently producing output
    ACTU_STOPPED,           ///< Stopped but still functional
    ACTU_ERROR,             ///< Error state, requires resetError()
    ACTU_DISABLED,          ///< Disabled via software, requires re-enable
    ACTU_E_STOP             ///< Emergency stop (latched, highest priority)
};

#endif // ACTUATOR_STATE_H
