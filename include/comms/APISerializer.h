/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef API_SERIALIZER_H
#define API_SERIALIZER_H

#include <ArduinoJson.h>
#include "core/ThrustStand.h"
#include "protocol/TestProtocolExecutor.h"

class ApiSerializer
{
public:
    // --- Calibration ---
    static void writeCalibrationConfig(JsonDocument &doc, const ThrustStand &ts);

    // --- RPM ---
    static void writeRpmOptions(JsonDocument &doc);

    // --- Live ---
    static void writeLiveSnapshot(JsonDocument &doc, const ThrustStand &ts);

    static void writeSystemState(JsonDocument &doc, const ThrustStand &ts);

    // --- Safety ---

    static void writeSafetyConfig(JsonDocument &doc, const ThrustStandSafety safety);

    static void writeBatteryPresetOptions(JsonDocument &doc);

    static void writeESCDriverOptions(JsonDocument &doc);

    // --- TEST ---

    static void writeTestConfig(JsonDocument &doc, const TestRunContext runContext);

    static void writeTestState(JsonDocument &doc, const ProtocolExecStatus ctx);
};

#endif // API_SERIALIZER_H