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
    stand.resetCumulativeData(); // Reset stats in the stand
    _status.step = 0;
    _step_start_ts = millis() + _step_accel_time_ms + 100;

    float maxThrottle = stand.getMaxThrottlePercent();
    if (maxThrottle < 100.f)
    {
        _status.total_steps = static_cast<int>(_max_total_steps * maxThrottle / 100.0f);
        // Safety: ensure at least one step if throttle > 0
        if (_status.total_steps < 1 && maxThrottle > 0.0f)
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
        write_csv_to_littlefs("/last_test.csv", _status.total_steps, _config.csvFormat.format);
    }
    else
    {
        ESP_LOGI(TAG, "Test stopped and no test data stored");
    }

    _status.running = false;
    _status.step = 0;
    _status.total_steps = 0;
    _status.progress = 0.0f;

    stand.resetCumulativeData();
    stand.setControlMode(ThrottleControlMode::MANUAL);
}

// Run the test (call in main loop)
void ThrustTestController::runTest()
{

    if (!_status.running)
        return; // No test running

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
    test_data[_status.step] = stand.getAverage();
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

bool ThrustTestController::write_csv_to_littlefs(const char *path,
                                                 uint32_t numberOfRecords,
                                                 const char *csvFormat)
{

    File f = LittleFS.open(path, "w");
    if (!f)
        return false;

    char decimalSep = csvFormat[0];
    char fieldSep = csvFormat[1];

    /* ---------- Metadata header ---------- */
    f.println("# ==================================");
    f.printf("# Application   : %s\n", THRUSTSTAND_APP_NAME);
    f.printf("# AppVersion    : %s\n", THRUSTSTAND_APP_VERSION);
    f.printf("# CSVVersion    : %s\n", THRUSTSTAND_CSV_VERSION);
    f.printf("# Build         : %s\n", THRUSTSTAND_BUILD_DATE);
    f.printf("# Motor Type    : %s\n", _config.metadata.motorType.c_str());
    f.printf("# ESC Type      : %s\n", _config.metadata.escType.c_str());
    f.printf("# Propeller Type: %s\n", _config.metadata.propellerType.c_str());
    // f.printf("# Records: %lu\n", (unsigned long)numberOfRecords);
    // f.printf("# Generated: %lu\n", (unsigned long)millis());
    f.println("# ==================================");

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
        "temp_C%c"
        "temp_max_C\n",
        fieldSep, fieldSep, fieldSep, fieldSep, fieldSep,
        fieldSep, fieldSep, fieldSep, fieldSep, fieldSep);

    for (uint32_t i = 0; i < numberOfRecords; i++)
    {
        const auto &d = test_data[i];

        f.print(i);
        f.print(fieldSep);
        printFloat(f, d.throttle, 1, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.thrust, 3, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.torque, 3, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.voltage, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.current, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.power, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.rpm, 0, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.thrust / d.power, 2, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.temperature, 1, decimalSep);
        f.print(fieldSep);
        printFloat(f, d.temperature_max, 2, decimalSep);
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
