/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACTUATOR_STATE_H
#define ACTUATOR_STATE_H

#include <stdint.h>

enum class ActuatorState : uint8_t
{
    ACTU_UNINITIALIZED = 0,
    ACTU_INITIALIZING,
    ACTU_READY,    // Enabled, idle, allowed to run
    ACTU_ACTIVE,   // Currently producing output
    ACTU_STOPPED,  // Explicit stop, still healthy
    ACTU_ERROR,    // Fault detected
    ACTU_DISABLED, // Software-disabled
    ACTU_E_STOP    // Emergency stop (latched)
};

#endif // ACTUATOR_STATE_H
