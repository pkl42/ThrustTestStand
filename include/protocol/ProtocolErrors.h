/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PROTOCOL_ERRORS_H
#define PROTOCOL_ERRORS_H

enum class ProtocolError
{
    Ok = 0,

    // Structural
    EmptyProtocol,
    TooManySteps,
    UnknownStepType,

    // Ranges
    ThrottleOutOfRange,
    RateOutOfRange,
    DwellOutOfRange,

    // Logic
    RampRateZero,
    TotalDurationExceeded,

    // Safety compatibility
    UnsafeCurrentLimit,
    UnsafeTemperatureLimit,

    // Safety Trigger from ThrustStand, could be E-Stop or other triggered Safety limits
    SafetyTriggerRaised,

    DataRecorderFull

};

const char *protocolErrorToString(ProtocolError e);

#endif // PROTOCOL_ERRORS_H
