/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "protocol/TestProtocolParser.h"
#include <Arduino.h>
#include <esp_log.h>

// --- Allowed keys arrays at file scope ---
static const char *rootKeys[] = {"id", "name", "version", "limits", "motion", "measurement", "steps"};
static const size_t rootKeys_count = sizeof(rootKeys) / sizeof(rootKeys[0]);

static const char *limitKeys[] = {"maxCurrentA", "maxTempC", "maxVoltageV", "minVoltageV", "maxDurationS", "maxThrustGF"};
static const size_t limitKeys_count = sizeof(limitKeys) / sizeof(limitKeys[0]);

static const char *motionKeys[] = {"smooth", "accelTimeMs", "decelTimeMs", "settleTimeMs"};
static const size_t motionKeys_count = sizeof(motionKeys) / sizeof(motionKeys[0]);

static const char *measurementKeys[] = {"enabled", "completion", "intervalS", "windowMs"};
static const size_t measurementKeys_count = sizeof(measurementKeys) / sizeof(measurementKeys[0]);

static const char *stepKeys[] = {"type", "throttle", "from", "to", "rate", "step", "dwellS", "motion", "measure"};
static const size_t stepKeys_count = sizeof(stepKeys) / sizeof(stepKeys[0]);

static const char *stepMotionKeys[] = {"smooth", "accelTimeMs", "decelTimeMs", "settleTimeMs"};
static const size_t stepMotionKeys_count = sizeof(stepMotionKeys) / sizeof(stepMotionKeys[0]);

static const char *stepMeasurementKeys[] = {"enabled", "completion", "intervalS", "windowMs"};
static const size_t stepMeasurementKeys_count = sizeof(stepMeasurementKeys) / sizeof(stepMeasurementKeys[0]);

// --- Helper: check unknown keys ---
bool TestProtocolParser::checkUnknownKeys(const JsonObjectConst &obj,
                                          const char *const *allowedKeys,
                                          size_t keyCount,
                                          String &outError)
{
    for (JsonPairConst kv : obj)
    {
        bool found = false;
        for (size_t i = 0; i < keyCount; ++i)
        {
            if (strcmp(kv.key().c_str(), allowedKeys[i]) == 0)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            outError = String("Unknown key: ") + kv.key().c_str();
            return false;
        }
    }
    return true;
}

// Helper to parse measurement completion from string
static MeasurementCompletion parseMeasurementCompletion(const char *completionStr)
{
    if (!completionStr)
        return MeasurementCompletion::OnStop;

    if (strcmp(completionStr, "fixedDuration") == 0)
        return MeasurementCompletion::FixedDuration;
    if (strcmp(completionStr, "periodic") == 0)
        return MeasurementCompletion::Periodic;
    if (strcmp(completionStr, "onStop") == 0)
        return MeasurementCompletion::OnStop;

    // Default fallback
    return MeasurementCompletion::OnStop;
}

// --- Hardened parser ---
bool TestProtocolParser::parse(const JsonDocument &doc,
                               TestProtocol &out,
                               String &outError)
{
    JsonObjectConst root = doc.as<JsonObjectConst>();

    // --- Check root keys ---
    if (!checkUnknownKeys(root, rootKeys, rootKeys_count, outError))
        return false;

    // --- Required protocol fields ---
    if (!root.containsKey("id") || !root.containsKey("name") || !root.containsKey("version"))
    {
        outError = "Missing required protocol fields (id, name, version)";
        return false;
    }
    strlcpy(out.id, root["id"], sizeof(out.id));
    strlcpy(out.name, root["name"], sizeof(out.name));
    strlcpy(out.version, root["version"], sizeof(out.version));

    out.estDurationS = 0.f;
    out.unitsTotal = 0;
    uint32_t estDurationMs = 0;

    // --- Limits (optional) ---
    if (root.containsKey("limits"))
    {
        JsonObjectConst limits = root["limits"].as<JsonObjectConst>();
        if (!checkUnknownKeys(limits, limitKeys, limitKeys_count, outError))
            return false;

        if (!limits.isNull())
        {
            out.limits.maxCurrentA = limits["maxCurrentA"] | out.limits.maxCurrentA;
            out.limits.maxTempC = limits["maxTempC"] | out.limits.maxTempC;
            out.limits.maxVoltageV = limits["maxVoltageV"] | out.limits.maxVoltageV;
            out.limits.minVoltageV = limits["minVoltageV"] | out.limits.minVoltageV;
            out.limits.maxDurationS = limits["maxDurationS"] | out.limits.maxDurationS;
            out.limits.maxThrustGF = limits["maxThrustGF"] | out.limits.maxThrustGF;
        }
    }

    // --- Motion defaults ---
    if (root.containsKey("motion"))
    {
        JsonObjectConst motion = root["motion"].as<JsonObjectConst>();
        if (!checkUnknownKeys(motion, motionKeys, motionKeys_count, outError))
            return false;

        out.motion.smooth = motion["smooth"] | out.motion.smooth;
        out.motion.accelTimeMs = motion["accelTimeMs"] | out.motion.accelTimeMs;
        out.motion.decelTimeMs = motion["decelTimeMs"] | out.motion.decelTimeMs;
        out.motion.settleTimeMs = motion["settleTimeMs"] | out.motion.settleTimeMs;
    }

    // --- Measurement defaults ---
    if (root.containsKey("measurement"))
    {
        JsonObjectConst measurement = root["measurement"].as<JsonObjectConst>();
        if (!checkUnknownKeys(measurement, measurementKeys, measurementKeys_count, outError))
            return false;

        out.measurement.enabled = measurement["enabled"] | true;
        if (measurement.containsKey("completion"))
            out.measurement.completion = parseMeasurementCompletion(measurement["completion"].as<const char *>());
        out.measurement.intervalS = measurement["intervalS"] | out.measurement.intervalS;
        out.measurement.windowMs = measurement["windowMs"] | out.measurement.windowMs;
    }

    // --- Steps ---
    if (!root.containsKey("steps") || !root["steps"].is<JsonArrayConst>())
    {
        outError = "Steps must be an array";
        return false;
    }

    JsonArrayConst steps = root["steps"].as<JsonArrayConst>();
    out.steps.clear();

    for (JsonObjectConst s : steps)
    {
        if (!checkUnknownKeys(s, stepKeys, stepKeys_count, outError))
            return false;

        if (!s.containsKey("type"))
        {
            outError = "Step missing type";
            return false;
        }

        TestStep step{};
        step.motion = out.motion;
        step.measure = out.measurement;

        const char *type = s["type"];

        if (strcmp(type, "set") == 0)
        {
            step.type = TestStepType::Set;
            step.throttleTo = s["throttle"] | 0.0f;
            float dwellS = s["dwellS"] | 0.0f;
            step.dwellMs = static_cast<uint32_t>(dwellS * 1000.0f);
            estDurationMs += step.dwellMs;
            out.unitsTotal++;
        }
        else if (strcmp(type, "hold") == 0)
        {
            step.type = TestStepType::Hold;
            step.throttleTo = s["throttle"] | 0.0f;
            float dwellS = s["dwellS"] | 0.0f;
            step.dwellMs = static_cast<uint32_t>(dwellS * 1000.0f);
            estDurationMs += step.dwellMs;
            out.unitsTotal++;
        }
        else if (strcmp(type, "ramp") == 0)
        {
            step.type = TestStepType::Ramp;
            step.throttleFrom = s["from"] | 0.0f;
            step.throttleTo = s["to"] | 0.0f;
            step.rampRatePctPerS = s["rate"] | 0.0f;
            estDurationMs += (step.throttleTo - step.throttleFrom) / step.rampRatePctPerS * 1000;
            out.unitsTotal++;
        }
        else if (strcmp(type, "stepSweep") == 0)
        {
            step.type = TestStepType::StepSweep;
            step.throttleFrom = s["from"] | 0.0f;
            step.throttleTo = s["to"] | 0.0f;
            step.stepPct = s["step"] | 1.0f;
            float dwellS = s["dwellS"] | 0.0f;
            step.dwellMs = static_cast<uint32_t>(dwellS * 1000.0f);
            estDurationMs += step.dwellMs;
            // calculate units
            float span = fabs(step.throttleTo - step.throttleFrom);
            float stepSize = fabs(step.stepPct);
            uint16_t unitsTotal = 1;

            if (stepSize > 0.0f)
            {
                // Inclusive sweep: 0,5,10,...,100 → 21 points
                uint32_t points =
                    static_cast<uint32_t>(floor(span / stepSize)) + 1;

                unitsTotal += max(points, 1u);
            }

            out.unitsTotal += unitsTotal;
            estDurationMs += (step.dwellMs * unitsTotal);
        }
        else
        {
            outError = String("Unknown step type: ") + type;
            return false;
        }

        // --- Optional per-step motion ---
        if (s.containsKey("motion"))
        {
            JsonObjectConst sm = s["motion"].as<JsonObjectConst>();
            if (!checkUnknownKeys(sm, stepMotionKeys, stepMotionKeys_count, outError))
                return false;

            step.motion.smooth = sm["smooth"] | step.motion.smooth;
            step.motion.accelTimeMs = sm["accelTimeMs"] | step.motion.accelTimeMs;
            step.motion.decelTimeMs = sm["decelTimeMs"] | step.motion.decelTimeMs;
            step.motion.settleTimeMs = sm["settleTimeMs"] | step.motion.settleTimeMs;
        }

        // --- Optional per-step measurement ---
        if (s.containsKey("measure"))
        {
            JsonObjectConst smm = s["measure"].as<JsonObjectConst>();
            if (!checkUnknownKeys(smm, stepMeasurementKeys, stepMeasurementKeys_count, outError))
                return false;

            step.measure.enabled = smm["enabled"] | step.measure.enabled;
            if (smm.containsKey("completion"))
                step.measure.completion = parseMeasurementCompletion(smm["completion"].as<const char *>());
            step.measure.intervalS = smm["intervalS"] | step.measure.intervalS;
            step.measure.windowMs = smm["windowMs"] | step.measure.windowMs;
        }

        out.steps.push_back(step);
    }

    if (out.steps.empty())
    {
        outError = "Protocol has no steps";
        return false;
    }
    out.estDurationS = estDurationMs / 1000.;

    return true;
}
