#include <Preferences.h>
#include "ThrustStand.h"
#include "ThrustTestController.h"
#include "esp_log.h"

const char *ThrustTestController::TAG = "ThrustTest";

constexpr unsigned long SETTLE_MS = 200; // delay when motor reached target speed until sensor data collection starts

static Preferences prefs;

// Constructor
ThrustTestController::ThrustTestController(ThrustStand &stand)
    : stand(stand)
{
}

void ThrustTestController::set_timing(
    uint32_t max_totalSteps,
    uint32_t stepAccelTimeMs,
    uint32_t stepTimeMs,
    uint32_t decelTimeMs)
{
    this->_max_total_steps = max_totalSteps;
    this->_step_accel_time_ms = stepAccelTimeMs;
    this->_step_time_ms = stepTimeMs;
    this->_decel_time_ms = decelTimeMs;
};

// Start the test
bool ThrustTestController::startTest()
{
    if (_status.running)
        return false;

    if (_config.metadata.motorType.isEmpty() || _config.metadata.propellerType.isEmpty())
        return false;

    bool success = stand.tareSensors();
    stand.setControlMode(ThrottleControlMode::AUTOMATIC);
    stand.resetAccumulativeData(); // Reset stats in the stand
    _status.step = 0;
    _step_start_ts = millis() + _step_accel_time_ms + 100;

    ThrustStandSafety standSafety = stand.getStandSafety();

    if (standSafety.limits.maxThrottlePercent < 100.f)
    {
        _status.total_steps = static_cast<int>(_max_total_steps * standSafety.limits.maxThrottlePercent / 100.0f);
        // Safety: ensure at least one step if throttle > 0
        if (_status.total_steps < 1 && standSafety.limits.maxThrottlePercent > 0.0f)
            _status.total_steps = 1;
    }
    else
    {
        _status.total_steps = _max_total_steps;
    }
    _status.running = true;
    updateProgress();
    ESP_LOGI(TAG, "Test started with %u steps", _status.total_steps);
    return true;
}

void ThrustTestController::stopTest(bool storeTestData)
{
    stand.setControlMode(ThrottleControlMode::MANUAL);
    if (!_status.running)
        return;

    stand.setThrottle(0., true, _decel_time_ms, ThrottleSource::TEST);

    if (storeTestData)
    {
        ESP_LOGI(TAG, "Test completed and test data stored");
        exportMeanCsv("/last_test_mean.csv", _status.total_steps, _config.csvFormat.format);
        exportStatisticsCsv("/last_test_stats.csv", _status.total_steps, _config.csvFormat.format);
    }
    else
    {
        ESP_LOGI(TAG, "Test stopped and no test data stored");
    }

    _status.running = false;
    _status.step = 0;
    _status.total_steps = 0;
    _status.progress = 0.0f;

    stand.resetAccumulativeData();
    stand.setControlMode(ThrottleControlMode::MANUAL);
}

// Run the test (called in main.cpp: task Sensor & Motor Task (Core 0)
void ThrustTestController::runTest()
{
    // ---------- NO TEST RUNNING ----------
    if (!_status.running)
        return; // No test running

    // ---------- SAFETY CHECK ----------
    const ThrustStandSafety safety = stand.getStandSafety();
    if (safety.state.tripped)
    {
        // Stop test immediately if tripped
        stopTest(true); // or false depending if you want to finalize logging
        ESP_LOGW(TAG, "Test halted: Safety trip active (%s @ %.2f)",
                 stand.safetyTripReasonToString(safety.state.reason),
                 safety.state.tripValue);
        return;
    }

    unsigned long now = millis();

    // ---------- WAIT FOR TARGET SPEED ----------
    if (!_accumulating)
    {

        if (!stand.isMotorAtTargetSpeed())
        {
            // Target not yet reached (or lost again)
            _target_reached_ts = 0;
            return;
        }

        // Target speed just reached → start settle timer
        if (_target_reached_ts == 0)
        {
            _target_reached_ts = now;
            return;
        }

        // Still settling
        if (now - _target_reached_ts < SETTLE_MS)
            return;

        // ---------- STEADY STATE CONFIRMED ----------
        stand.startAccumulation();
        _accumulating = true;
        _step_start_ts = now; // redefine start as steady-state start
        return;
    }

    // ---------- STEADY STATE / ACCUMULATION ----------
    if (now < _step_start_ts + _step_time_ms)
        return;

    // ---------- STEP COMPLETE ----------
    stand.stopAccumulation();
    test_data[_status.step] = stand.getAccuDataSet();
    _status.step++;
    updateProgress();

    if (_status.step > _status.total_steps)
    {
        stopTest(true);
        return;
    }

    // ---------- PREPARE NEXT STEP ----------

    float throttle = (float)_status.step / _max_total_steps * 100.0f;

    _accumulating = false;
    _target_reached_ts = 0; // IMPORTANT: reset for next step

    stand.setThrottle(throttle, true, _step_accel_time_ms, ThrottleSource::TEST);
}

void ThrustTestController::printFloat(File &f, float value, int precision, char decimalSep)
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

void ThrustTestController::writeCsvMetadata(File &f)
{

    /* ---------- Metadata header ---------- */
    f.println("# ==================================");
    f.printf("# Application   : %s\n", THRUSTSTAND_APP_NAME);
    f.printf("# AppVersion    : %s\n", THRUSTSTAND_APP_VERSION);
    f.printf("# CSVVersion    : %s\n", THRUSTSTAND_CSV_VERSION);
    f.printf("# Build         : %s\n", THRUSTSTAND_BUILD_DATE);
    f.printf("# Motor Type    : %s\n", _config.metadata.motorType.c_str());
    f.printf("# ESC Type      : %s\n", _config.metadata.escType.c_str());
    f.printf("# Propeller Type: %s\n", _config.metadata.propellerType.c_str());
    f.println("#");
    f.println("# Derived values:");
    f.println("# power_W        = voltage_mean * current_mean");
    f.println("# thrust_ratio   = thrust_mean / power_W");
    f.println("# ==================================");
}

bool ThrustTestController::exportMeanCsv(const char *path,
                                         uint32_t numberOfRecords,
                                         const char *csvFormat)
{

    File f = LittleFS.open(path, "w");
    if (!f)
        return false;

    char decimalSep = csvFormat[0];
    char fieldSep = csvFormat[1];

    writeCsvMetadata(f);

    f.printf(
        "step%c"
        "throttle_pct%c"
        "thrust_g%c"
        "torque_Ncm%c"
        "voltage_V%c"
        "current_A%c"
        "power_W%c"
        "rpm%c"
        "g/W%c"
        "temp_C%c\n",
        fieldSep, fieldSep, fieldSep, fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep);

    for (uint32_t i = 0; i < numberOfRecords; i++)
    {
        const auto &d = test_data[i];

        f.print(i);
        f.print(fieldSep);
        printFloat(f, d.throttle, 1, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.thrust.mean, 3, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.torque.mean, 3, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.voltage.mean, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.current.mean, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, (d.current.mean * d.voltage.mean), 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.rpm.mean, 0, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.thrust.mean / (d.current.mean * d.voltage.mean), 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.temperature.mean, 1, decimalSep);
        f.println();
    }

    f.close();
    return true;
}

void ThrustTestController::printStats(File &f,
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

bool ThrustTestController::exportStatisticsCsv(const char *path,
                                               uint32_t numberOfRecords,
                                               const char *csvFormat)
{
    File f = LittleFS.open(path, "w");
    if (!f)
        return false;

    const char decimalSep = csvFormat[0];
    const char fieldSep = csvFormat[1];

    /* ---------- Metadata ---------- */
    writeCsvMetadata(f);

    /* ---------- Header ---------- */
    f.printf(
        "step%c"
        "throttle_pct%c"

        "thrust_mean_N%cthrust_min_N%cthrust_max_N%cthrust_std_N%c"
        "torque_mean_Ncm%ctorque_min_Ncm%ctorque_max_Ncm%ctorque_std_Ncm%c"

        "voltage_mean_V%cvoltage_min_V%cvoltage_max_V%cvoltage_std_V%c"
        "current_mean_A%ccurrent_min_A%ccurrent_max_A%ccurrent_std_A%c"

        "power_mean_W%c"
        "rpm_mean%crpm_min%crpm_max%crpm_std%c"

        "thrust_ratio%c"
        "temp_mean_C%ctemp_min_C%ctemp_max_C%ctemp_std_C\n",

        fieldSep,
        fieldSep,

        fieldSep, fieldSep, fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep,

        fieldSep, fieldSep, fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep,

        fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep,

        fieldSep,
        fieldSep, fieldSep, fieldSep);

    /* ---------- Data ---------- */
    for (uint32_t i = 0; i < numberOfRecords; i++)
    {
        const auto &d = test_data[i];

        f.print(i);
        f.print(fieldSep);
        printFloat(f, d.throttle, 1, decimalSep);
        f.print(fieldSep);

        printStats(f, d.thrust, 3, decimalSep, fieldSep);
        f.print(fieldSep);
        printStats(f, d.torque, 3, decimalSep, fieldSep);
        f.print(fieldSep);
        printStats(f, d.voltage, 2, decimalSep, fieldSep);
        f.print(fieldSep);
        printStats(f, d.current, 2, decimalSep, fieldSep);
        f.print(fieldSep);

        /* power from mean values */
        float power = d.voltage.mean * d.current.mean;
        printFloat(f, power, 2, decimalSep);
        f.print(fieldSep);

        printStats(f, d.rpm, 0, decimalSep, fieldSep);
        f.print(fieldSep);

        /* thrust ratio from mean values */
        float thrustRatio = (power > 0.0f) ? (d.thrust.mean / power) : 0.0f;
        printFloat(f, thrustRatio, 2, decimalSep);
        f.print(fieldSep);

        printStats(f, d.temperature, 1, decimalSep, fieldSep);

        f.println();
    }

    f.close();
    return true;
}

void ThrustTestController::updateProgress()
{
    if (!_status.running || _status.total_steps == 0)
    {
        _status.progress = 0.0f;
        return;
    }

    _status.progress =
        (float)_status.step / (float)(_status.total_steps) * 100.0f;

    if (_status.progress > 100.0f)
        _status.progress = 100.0f;
}

bool ThrustTestController::setMetadata(const String &motor,
                                       const String &escType,
                                       const String &prop)
{
    if (_status.running)
    {
        return false; // metadata is immutable while running
    }

    _config.metadata.motorType = motor;
    _config.metadata.escType = escType;
    _config.metadata.propellerType = prop;
    saveConfig();
    return true;
}

bool ThrustTestController::setCsvFormat(const char *format, bool saveConfigFlag)
{
    if (!format || strlen(format) != 2)
        return false;

    ESP_LOGI(TAG, "setCsvFormat: %s", format);

    _config.csvFormat.format[0] = format[0];
    _config.csvFormat.format[1] = format[1];
    _config.csvFormat.format[2] = '\0';
    if (saveConfigFlag)
        saveConfig();
    return true;
}

void ThrustTestController::begin()
{
    loadConfig();
}

void ThrustTestController::loadConfig()
{
    float interimValue;
    prefs.begin("test", false); // false=write mode in case the values as not yet set the will be written via putFloat, ....

    if (!prefs.isKey("motorType"))
        prefs.putString("motorType", "Motor Type");
    if (!prefs.isKey("escType"))
        prefs.putString("escType", "ESC Type");
    if (!prefs.isKey("propellerType"))
        prefs.putString("propellerType", "Propeller Type");

    _config.metadata.motorType = prefs.getString("motorType");
    _config.metadata.escType = prefs.getString("escType");
    _config.metadata.propellerType = prefs.getString("propellerType");

    if (!prefs.isKey("csvFormat"))
        prefs.putString("csvFormat", ",;"); // German Excel Style

    String fmt = prefs.getString("csvFormat", ".;");
    setCsvFormat(fmt.c_str(), false); // reuse validation logic

    prefs.end();
}

void ThrustTestController::saveConfig()
{
    prefs.begin("test", false);
    prefs.putString("motorType", _config.metadata.motorType);
    prefs.putString("escType", _config.metadata.escType);
    prefs.putString("propellerType", _config.metadata.propellerType);

    prefs.putString("csvFormat", _config.csvFormat.format);

    prefs.end();
}
