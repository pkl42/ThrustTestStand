/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "comms/APISerializer.h"
#include "core/Config.h"

void ApiSerializer::writeCalibrationConfig(JsonDocument &doc, const ThrustStand &ts)
{

    doc["version"] = THRUSTSTAND_APP_VERSION;

    doc["thrust"]["cal"] = ts.getThrustCalFactor();

    auto torque = ts.getTorqueCalibration();
    doc["torque"]["cal1"] = torque.cal1;
    doc["torque"]["cal2"] = torque.cal2;
    doc["torque"]["distance_mm"] = torque.distanceMM;

    doc["current"]["sensitivity"] = ts.getCurrentSensitivity();
    doc["voltage"]["calibration"] = ts.getVoltageCalibrationFactor();

    auto escConfig = ts.getESCDriverConfig();
    doc["motor"]["pwm_min_us"] = escConfig.minPulseUs;
    doc["motor"]["pwm_max_us"] = escConfig.maxPulseUs;
    doc["motor"]["min_spin_percent"] = escConfig.minSpinPercentage;
    //
    doc["motor"]["disarm_timeout_s"] = ts.getAutoDisarmTimeOut();
    doc["motor"]["esc_driver_type"] = static_cast<uint8_t>(ts.getESCDriver());
    //
    auto temperature = ts.getTemperatureCalibration();
    doc["temp"]["coefficient"] = temperature.coefficient;
    doc["temp"]["offset"] = temperature.offset;

    auto rpm = ts.getRPMUICalibration();
    doc["rpm"]["pulsesPerRevolution"] = rpm.pulsesPerRevolution;
    doc["rpm"]["responseMode"] = static_cast<uint8_t>(rpm.responseMode);
    doc["rpm"]["filterLevel"] = static_cast<uint8_t>(rpm.filterLevel);
    doc["rpm"]["smoothing"] = static_cast<uint8_t>(rpm.smoothing);
}

void ApiSerializer::writeSystemState(JsonDocument &doc, const ThrustStand &ts)
{
    const ThrustStandState s = ts.getState();

    doc["thrust"] = static_cast<uint8_t>(s.thrustSensor);
    doc["torque"] = static_cast<uint8_t>(s.torqueSensor);
    doc["temperature"] = static_cast<uint8_t>(s.tempSensor);
    doc["rpm"] = static_cast<uint8_t>(s.rpmSensor);
    doc["current"] = static_cast<uint8_t>(s.currentSensor);
    doc["voltage"] = static_cast<uint8_t>(s.voltageSensor);
    doc["motor"] = static_cast<uint8_t>(s.motor);
    doc["motor_mount"] = static_cast<uint8_t>(s.motorMountState);
    doc["mode"] = systemModeToString(ts.getSystemMode());
}

void ApiSerializer::writeLiveSnapshot(JsonDocument &doc, const ThrustStand &ts)
{
    const test_data_t d = ts.getCurrentDataSet();
    const SafetyState safetyState = ts.getSafetyState();

    doc["throttle_cmd"] = d.throttle_cmd;
    doc["throttle_act"] = d.throttle_act;
    doc["thrust"] = d.thrust;
    doc["torque"] = d.torque;
    doc["torqueCell1"] = d.torque_cell_1;
    doc["torqueCell2"] = d.torque_cell_2;

    doc["voltage"] = d.voltage;
    doc["current"] = d.current;

    doc["power"] = d.power;
    doc["thrust_ratio"] = d.thrust_ratio;

    doc["rpm"] = d.rpm;
    doc["temperature"] = d.temperature;

    // Safety state
    doc["safety_tripped"] = safetyState.tripped;
    doc["safety_trip_value"] = safetyState.tripValue;
    doc["safety_trip_reason"] =
        safetyTripReasonToString(safetyState.reason);
    doc["safety_trip_source"] = safetyTripSourceToString(safetyState.source);
}

void ApiSerializer::writeSafetyConfig(JsonDocument &doc, const ThrustStandSafety safety)
{

    // Limits
    doc["limits"]["throttle_percent"] = safety.coreLimits.maxThrottlePercent;
    doc["limits"]["current_a"] = safety.coreLimits.maxCurrentA;
    doc["limits"]["voltage_max_v"] = safety.coreLimits.maxVoltageV;
    doc["limits"]["voltage_min_v"] = safety.coreLimits.minVoltageV;
    doc["limits"]["thrust_gf"] = safety.coreLimits.maxThrustGF;
    doc["limits"]["battery_cells"] = batteryPresetToCells(safety.coreLimits.batteryPreset);
    doc["limits"]["temperature_c"] = safety.coreLimits.maxTemperatureC;
}

void ApiSerializer::writeBatteryPresetOptions(JsonDocument &doc)
{
    JsonArray arr = doc.createNestedArray("presets");

    auto addPreset = [&](BatteryPreset preset,
                         const char *label,
                         float minV,
                         float maxV)
    {
        JsonObject obj = arr.createNestedObject();
        const uint8_t cells = static_cast<uint8_t>(preset);

        obj["id"] = cells;
        obj["label"] = label;
        obj["cells"] = cells;
        obj["voltage_min_v"] = minV;
        obj["voltage_max_v"] = maxV;
    };

    /* ---------- NONE / Calibration ---------- */
    addPreset(
        BatteryPreset::BATTERY_PRESET_NONE,
        batteryPresetToString(BatteryPreset::BATTERY_PRESET_NONE),
        -0.01f, // Slightly below zero to avoid noise-triggered trips when disabled
        VOLTAGE_SENSOR_MAX);

    /* ---------- LiPo presets ---------- */
    addPreset(
        BatteryPreset::BATTERY_PRESET_3S,
        batteryPresetToString(BatteryPreset::BATTERY_PRESET_3S),
        3 * 3.2f,
        3 * 4.2f);

    addPreset(
        BatteryPreset::BATTERY_PRESET_4S,
        batteryPresetToString(BatteryPreset::BATTERY_PRESET_4S),
        4 * 3.2f,
        4 * 4.2f);

    addPreset(
        BatteryPreset::BATTERY_PRESET_5S,
        batteryPresetToString(BatteryPreset::BATTERY_PRESET_5S),
        5 * 3.2f,
        5 * 4.2f);

    addPreset(
        BatteryPreset::BATTERY_PRESET_6S,
        batteryPresetToString(BatteryPreset::BATTERY_PRESET_6S),
        6 * 3.2f,
        6 * 4.2f);
}

void ApiSerializer::writeESCDriverOptions(JsonDocument &doc)
{
    JsonArray arr = doc.createNestedArray("esc_drivers");

    auto addDriver = [&](EscDriverType type, const char *label)
    {
        JsonObject obj = arr.createNestedObject();
        obj["id"] = static_cast<uint8_t>(type);
        obj["label"] = label;
    };

    // ---------- NONE ----------
    addDriver(EscDriverType::ESC_DRIVER_NONE, escDriverToString(EscDriverType::ESC_DRIVER_NONE));

    // ---------- PWM ----------
    addDriver(EscDriverType::ESC_DRIVER_PWM, escDriverToString(EscDriverType::ESC_DRIVER_PWM));

    // ---------- D-Shot variants ----------

    addDriver(EscDriverType::ESC_DRIVER_DSHOT300, escDriverToString(EscDriverType::ESC_DRIVER_DSHOT300));
    addDriver(EscDriverType::ESC_DRIVER_DSHOT600, escDriverToString(EscDriverType::ESC_DRIVER_DSHOT600));
    addDriver(EscDriverType::ESC_DRIVER_DSHOT1200, escDriverToString(EscDriverType::ESC_DRIVER_DSHOT1200));
}

void ApiSerializer::writeRpmOptions(JsonDocument &doc)
{
    JsonObject rpm = doc["rpm"].to<JsonObject>();
    JsonObject opts = rpm["options"].to<JsonObject>();

    JsonArray arr = opts.createNestedArray("filter");

    auto addLov1 = [&](RpmFilterLevel type, const char *label)
    {
        JsonObject obj = arr.createNestedObject();
        obj["id"] = static_cast<uint8_t>(type);
        obj["label"] = label;
    };

    addLov1(RpmFilterLevel::WEAK, rpmFilterToString(RpmFilterLevel::WEAK));
    addLov1(RpmFilterLevel::NORMAL, rpmFilterToString(RpmFilterLevel::NORMAL));
    addLov1(RpmFilterLevel::STRONG, rpmFilterToString(RpmFilterLevel::STRONG));

    // ------------------------ //

    JsonArray arr2 = opts.createNestedArray("response");

    auto addLov2 = [&](RpmResponseMode type, const char *label)
    {
        JsonObject obj = arr2.createNestedObject();
        obj["id"] = static_cast<uint8_t>(type);
        obj["label"] = label;
    };

    addLov2(RpmResponseMode::FAST, rpmResponseToString(RpmResponseMode::FAST));
    addLov2(RpmResponseMode::BALANCED, rpmResponseToString(RpmResponseMode::BALANCED));
    addLov2(RpmResponseMode::STABLE, rpmResponseToString(RpmResponseMode::STABLE));

    // ------------------------ //

    JsonArray arr3 = opts.createNestedArray("smoothing");

    auto addLov3 = [&](RpmSmoothing type, const char *label)
    {
        JsonObject obj = arr3.createNestedObject();
        obj["id"] = static_cast<uint8_t>(type);
        obj["label"] = label;
    };

    addLov3(RpmSmoothing::RESPONSIVE, rpmSmoothingToString(RpmSmoothing::RESPONSIVE));
    addLov3(RpmSmoothing::BALANCED, rpmSmoothingToString(RpmSmoothing::BALANCED));
    addLov3(RpmSmoothing::SMOOTH, rpmSmoothingToString(RpmSmoothing::SMOOTH));
}

void ApiSerializer::writeTestState(JsonDocument &doc, const ProtocolExecStatus ctx)
{

    doc["steps_total"] = ctx.totalSteps;
    doc["unitsTotal"] = ctx.progressUnitsTotal;
    doc["running"] = ctx.running;
    if (ctx.execState != ExecState::Idle && ctx.totalSteps > 0)
    {
        doc["step"] = (ctx.currentStepIndex >= ctx.totalSteps) ? ctx.totalSteps : (ctx.currentStepIndex + 1);
        doc["unitsDone"] = (ctx.progressUnitsDone >= ctx.progressUnitsTotal) ? ctx.progressUnitsTotal : (ctx.progressUnitsDone + 1);
    }
    else
    {
        doc["step"] = 0;
        doc["unitsDone"] = 0;
    }

    doc["progress"] = ctx.progressPct;

    // doc["thrust_samples"] = c.thrust.n; PKL 20260327 decomissioned
    doc["durationS"] = ctx.durationS;
    doc["estDurationS"] = ctx.estDurationS;
    doc["execState"] = static_cast<uint8_t>(ctx.execState);
    doc["safetyState_tripped"] = ctx.safetyState.tripped;
}

void ApiSerializer::writeTestConfig(JsonDocument &doc, const TestRunContext runContext)
{
    doc["motorType"] = runContext.motorType;
    doc["escType"] = runContext.escType;
    doc["propellerType"] = runContext.propellerType;
    doc["csvFormat"] = runContext.csvFormat;
    doc["protocolID"] = runContext.protocolID;
    doc["protocolVersion"] = runContext.protocolVersion;
}
