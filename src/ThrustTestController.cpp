#include <Preferences.h>
#include <LittleFS.h>

#include "ThrustTestController.h"

#include "esp_log.h"
#include "Version.h"

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

    if (metadata.motorType.isEmpty() || metadata.propellerType.isEmpty())
        return false;

    stand.tareSensors();
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
        write_csv_to_littlefs("/last_test.csv", _status.total_steps);
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

bool ThrustTestController::write_csv_to_littlefs(const char *path, uint32_t numberOfRecords)
{
    File f = LittleFS.open(path, "w");
    if (!f)
        return false;

    /* ---------- Metadata header ---------- */
    f.println("# ==================================");
    f.printf("# Application: %s\n", THRUSTSTAND_APP_NAME);
    f.printf("# AppVersion: %s\n", THRUSTSTAND_APP_VERSION);
    f.printf("# CSVVersion: %s\n", THRUSTSTAND_CSV_VERSION);
    f.printf("# Build: %s\n", THRUSTSTAND_BUILD_DATE);
    f.printf("# MotorType: %s\n", metadata.motorType.c_str());
    f.printf("# ESCType: %s\n", metadata.escType.c_str());
    f.printf("# PropellerType: %s\n", metadata.propellerType.c_str());
    // f.printf("# Records: %lu\n", (unsigned long)numberOfRecords);
    // f.printf("# Generated: %lu\n", (unsigned long)millis());
    f.println("# ==================================");

    f.println(
        "step,"
        "throttle_pct,"
        "thrust_g,"
        "torque_Ncm,"
        "voltage_V,"
        "current_A,"
        "power_W,"
        "rpm,"
        "g/W,"
        "temp_C,"
        "temp_max_C");

    for (int i = 0; i < numberOfRecords + 1; i++)
    {
        const auto &d = test_data[i];
        f.printf(
            "%d,%.1f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,,%.2f\n",
            i,
            d.throttle,
            d.thrust,
            d.torque,
            d.voltage,
            d.current,
            d.power,
            d.rpm,
            (d.thrust / d.power),
            d.temperature,
            d.temperature_max

        );
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

    metadata.motorType = motor;
    metadata.escType = escType;
    metadata.propellerType = prop;
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

    metadata.motorType = prefs.getString("motorType");
    metadata.escType = prefs.getString("escType");
    metadata.propellerType = prefs.getString("propellerType");

    prefs.end();
}

void ThrustTestController::saveConfig()
{
    prefs.begin("test", false);
    prefs.putString("motorType", metadata.motorType);
    prefs.putString("escType", metadata.escType);
    prefs.putString("propellerType", metadata.propellerType);

    prefs.end();
}
