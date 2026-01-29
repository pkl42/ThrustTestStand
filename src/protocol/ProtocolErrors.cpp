/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "protocol/ProtocolErrors.h"

const char *protocolErrorToString(ProtocolError e)
{
    switch (e)
    {
    case ProtocolError::Ok:
        return "OK";
    case ProtocolError::EmptyProtocol:
        return "Protocol has no steps";
    case ProtocolError::TooManySteps:
        return "Too many steps";
    case ProtocolError::UnknownStepType:
        return "Unknown step type";
    case ProtocolError::ThrottleOutOfRange:
        return "Throttle out of range";
    case ProtocolError::RateOutOfRange:
        return "Ramp rate out of range";
    case ProtocolError::DwellOutOfRange:
        return "Dwell time out of range";
    case ProtocolError::RampRateZero:
        return "Ramp rate must be > 0";
    case ProtocolError::TotalDurationExceeded:
        return "Total duration exceeded";
    case ProtocolError::UnsafeCurrentLimit:
        return "Unsafe current limit";
    case ProtocolError::UnsafeTemperatureLimit:
        return "Unsafe temperature limit";
    case ProtocolError::SafetyTriggerRaised:
        return "Safety Trigger raised";
    case ProtocolError::DataRecorderFull:
        return "Data Recorder is full";

    default:
        return "Unknown error";
    }
}
