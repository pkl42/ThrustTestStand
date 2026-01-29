/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "recorder/TestDataRecorder.h"
#include "core/ThrustStand.h"
#include <Preferences.h>
#include <Arduino.h>

const char *TestDataRecorder::TAG = "TestDataRecorder";

TestDataRecorder::TestDataRecorder(ThrustStand &stand)
    : _stand(stand)
{
    reset();
}

bool TestDataRecorder::begin()
{
    reset();
    return true;
}

void TestDataRecorder::reset(uint32_t ts_offset)
{
    _recording = false;
    _sampleIndex = INVALID_REC_INDEX;
    _ts_offset = ts_offset;
    memset(_data, 0, sizeof(_data));
}

void TestDataRecorder::beginSampling()
{
    _stand.startAccumulation();
    _window.begin(_measureConfig.windowMs);
    _lastSampleMs = millis();
}

void TestDataRecorder::endSampling()
{
    _stand.stopAccumulation();
    _window.stop();

    _data[_sampleIndex] = _stand.getAccuDataSet();
    _data[_sampleIndex].t_ms = _lastSampleMs - _ts_offset;
}

void TestDataRecorder::stop()
{
    if (!_recording)
        return;

    if (_sampling)
    {
        endSampling();
        _sampling = false;
    }

    _recording = false;
}

void TestDataRecorder::trigger(const MeasurementConfig &mc)
{
    _measureConfig = mc;
    if (!_measureConfig.enabled)
    {
        stop();
        return;
    }

    start();
}

void TestDataRecorder::start()
{
    if (_recording)
        return;

    if (isFull())
        return;

    _sampleIndex++;
    _data[_sampleIndex] = {};

    _recording = true;
    _sampling = true;

    beginSampling(); // use helper

    if (_measureConfig.completion == MeasurementCompletion::Periodic)
    {
        _nextIntervalMs = millis();
        if (_measureConfig.intervalS * 1000 <= _measureConfig.windowMs)
        {
            _measureConfig.windowMs = _measureConfig.intervalS * 995; // sampling time must be smaller than interval time
        }
    }
}

void TestDataRecorder::update()
{
    if (!_recording || !_measureConfig.enabled)
        return;

    unsigned long now = millis();

    switch (_measureConfig.completion)
    {
    case MeasurementCompletion::OnStop:
        // nothing to do
        return;

    case MeasurementCompletion::FixedDuration:
        if (_sampling && _window.expired())
        {
            endSampling();
            _sampling = false;
            _recording = false;
        }
        return;

    case MeasurementCompletion::Periodic:
        break;
    }

    // --- Periodic logic ---
    if (_sampling)
    {
        if (!_window.expired())
            return;

        endSampling();
        _sampling = false;

        _nextIntervalMs = now + _measureConfig.intervalS * 1000 - _measureConfig.windowMs;
    }
    else
    {
        if (now < _nextIntervalMs)
            return;

        if (isFull())
            return;

        _sampleIndex++;
        _data[_sampleIndex] = {};

        beginSampling();
        _sampling = true;
    }
}

void TestDataRecorder::printFloat(File &f, float value, int precision, char decimalSep)
{
    char buf[32];
    dtostrf(value, 0, precision, buf);

    if (decimalSep != '.')
    {
        for (char *p = buf; *p; ++p)
        {
            if (*p == '.')
            {
                *p = decimalSep;
                break;
            }
        }
    }

    f.print(buf);
}

void TestDataRecorder::printStats(File &f,
                                  const sensor_stats_t &s,
                                  uint8_t decimals,
                                  char decimalSep,
                                  char fieldSep)
{
    printFloat(f, s.mean, decimals, decimalSep);
    f.print(fieldSep);
    printFloat(f, s.min, decimals, decimalSep);
    f.print(fieldSep);
    printFloat(f, s.max, decimals, decimalSep);
    f.print(fieldSep);

    float stddev = (s.n > 1) ? sqrtf(s.M2 / (s.n - 1)) : 0.0f;
    printFloat(f, stddev, decimals, decimalSep);
}

void TestDataRecorder::writeCsvMetadata(File &f, const TestRunContext &testContext,
                                        const ProtocolExecStatus &testResult)
{

    /* ---------- Metadata header ---------- */
    f.println("# ==================================");
    f.printf("# Application    : %s\n", THRUSTSTAND_APP_NAME);
    f.printf("# AppVersion     : %s\n", THRUSTSTAND_APP_VERSION);
    f.printf("# CSVVersion     : %s\n", THRUSTSTAND_CSV_VERSION);
    f.printf("# Build          : %s\n", THRUSTSTAND_BUILD_DATE);
    f.printf("# Motor Type     : %s\n", testContext.motorType.c_str());
    f.printf("# ESC Type       : %s\n", testContext.escType.c_str());
    f.printf("# Propeller Type : %s\n", testContext.propellerType.c_str());
    f.printf("# Protocol       : %s Version: %s\n", testContext.protocolID.c_str(), testContext.protocolVersion.c_str());
    f.printf("# Completion     : %s\n", execStateToString(testResult.execState));
    if (testResult.safetyState.tripped)
    {
        f.printf("# SafetyState    : source : %s - reason: %s - value: %f\n",
                 safetyTripSourceToString(testResult.safetyState.source),
                 safetyTripReasonToString(testResult.safetyState.reason),
                 testResult.safetyState.tripValue);
    }
    else
    {
        f.println("#");
    }
    f.printf("# Duration Actual: %0.f s - Duration Estimated: %0.f s\n", testResult.durationS, testResult.estDurationS);
    f.println("# ==================================");
}

bool TestDataRecorder::exportStatisticsCsv(const char *path,
                                           const TestRunContext &testContext,
                                           const ProtocolExecStatus &testResult)
{
    File f = LittleFS.open(path, "w");
    if (!f)
        return false;

    const char decimalSep = testContext.csvFormat[0];
    const char fieldSep = testContext.csvFormat[1];

    /* ---------- Metadata ---------- */
    writeCsvMetadata(f, testContext, testResult);

    /* ---------- Header ---------- */
    f.printf(
        "step%ctime_stamp_ms%c"
        "throttle_mean_pct%cthrottle_min_pct%cthrottle_max_pct%cthrottle_std_pct%c"

        "thrust_mean_N%cthrust_min_N%cthrust_max_N%cthrust_std_N%c"
        "torque_mean_Ncm%ctorque_min_Ncm%ctorque_max_Ncm%ctorque_std_Ncm%c"

        "voltage_mean_V%cvoltage_min_V%cvoltage_max_V%cvoltage_std_V%c"
        "current_mean_A%ccurrent_min_A%ccurrent_max_A%ccurrent_std_A%c"

        "power_mean_W%cpower_min_W%cpower_max_W%cpower_std_W%c"
        "rpm_mean%crpm_min%crpm_max%crpm_std%c"

        "thrust_ratio_mean%cthrust_ratio_min%cthrust_ratio_max%cthrust_ratio_std%c"
        "temp_mean_C%ctemp_min_C%ctemp_max_C%ctemp_std_C%c\n",

        fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep,

        fieldSep, fieldSep, fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep,

        fieldSep, fieldSep, fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep,

        fieldSep, fieldSep, fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep,

        fieldSep, fieldSep, fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep);

    /* ---------- Data ---------- */
    if (_sampleIndex == -1)
        return true;

    for (uint32_t i = 0; i <= _sampleIndex; ++i)
    {
        const auto &d = _data[i];

        f.print(i);
        f.print(fieldSep);
        f.print(d.t_ms);
        f.print(fieldSep);
        printStats(f, d.throttle, 1, decimalSep, fieldSep);
        f.print(fieldSep);

        printStats(f, d.thrust, 3, decimalSep, fieldSep);
        f.print(fieldSep);
        printStats(f, d.torque, 3, decimalSep, fieldSep);
        f.print(fieldSep);
        printStats(f, d.voltage, 2, decimalSep, fieldSep);
        f.print(fieldSep);
        printStats(f, d.current, 2, decimalSep, fieldSep);
        f.print(fieldSep);

        printStats(f, d.power, 2, decimalSep, fieldSep);
        f.print(fieldSep);

        printStats(f, d.rpm, 0, decimalSep, fieldSep);
        f.print(fieldSep);

        printStats(f, d.thrust_ratio, 2, decimalSep, fieldSep);
        f.print(fieldSep);

        printStats(f, d.temperature, 1, decimalSep, fieldSep);

        f.println();
    }

    f.close();
    return true;
}

bool TestDataRecorder::exportMeanCsv(const char *path,
                                     const TestRunContext &testContext,
                                     const ProtocolExecStatus &testResult)
{
    File f = LittleFS.open(path, "w");
    if (!f)
        return false;

    char decimalSep = testContext.csvFormat[0];
    char fieldSep = testContext.csvFormat[1];

    writeCsvMetadata(f, testContext, testResult);

    // CSV header
    f.printf(
        "step%c"
        "time_stamp_ms%c"
        "throttle_pct%c"
        "thrust_g%c"
        "torque_Ncm%c"
        "voltage_V%c"
        "current_A%c"
        "power_W%c"
        "rpm%c"
        "g/W%c"
        "temp_C%c\n",
        fieldSep, fieldSep, fieldSep, fieldSep, fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep, fieldSep);

    if (_sampleIndex == -1)
        return true;

    for (uint32_t i = 0; i <= _sampleIndex; ++i)
    {
        const auto &d = _data[i];

        f.print(i);
        f.print(fieldSep);
        f.print(d.t_ms);
        f.print(fieldSep);
        printFloat(f, d.throttle.mean, 1, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.thrust.mean, 3, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.torque.mean, 3, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.voltage.mean, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.current.mean, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.power.mean, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.rpm.mean, 0, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.thrust_ratio.mean, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.temperature.mean, 1, decimalSep);
        f.println();
    }

    f.close();
    return true;
}
