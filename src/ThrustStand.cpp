#include <Arduino.h>
#include <esp_log.h>
#include "pinOut.h"
#include <Preferences.h>
#include <nvs_flash.h> // deleting nvs flash
#include <cfloat>
#include "ThrustStand.h"

const char *ThrustStand::TAG = "ThrustStand";

static Preferences prefs;

// calibration of load cells
// 1. set calibration_value to 1 -> compile
// 2. put a known weight on load cell (weight) and note the output (measure)
// 3. calibration value is measuered(display)/weight

test_data_t ThrustStand::_currentDataSet = {};
test_data_accu_t ThrustStand::_cumDataSet = {};

ThrustStand::ThrustStand()
    : thrustSensor(HX711_DOUT_1_PIN, HX711_SCK_1_PIN),
      torqueSensor(HX711_DOUT_2_PIN, HX711_SCK_2_PIN,
                   HX711_DOUT_3_PIN, HX711_SCK_3_PIN),
      rpmSensor(),
      thermocoupleSensor(),
      currentSensor(),
      voltageSensor(),
      hwEstop(ESTOP_PIN),
      motor(MOTOR_ESC_PIN, 50, 13, 1)

{
    // other members can be initialized here
}

bool ThrustStand::init()
{

    loadConfig();

    if (!init_sensors())
    {

        return false;
    }

    if (!motor.begin())
    {

        return false;
    };
    motor.arm(); // Arm the ESC with a 2 second min throttle

    return true;
}

bool ThrustStand::init_sensors()
{
    bool initialize = true;
    ESP_LOGI(TAG, "Initializing Sensors ...");

    if (THRUST_SENSOR_INSTALLED && (thrustSensor.getState() == SensorState::SENSOR_UNINITIALIZED))
    {
        if (!thrustSensor.begin()) // calibration value already set by loadConfig()
            initialize = false;
    }

    if (TORQUE_SENSOR_INSTALLED)
    {
        if (torqueSensor.getState() != SensorState::SENSOR_READY)
        {
            if (!torqueSensor.begin()) // calibration value already set by loadConfig()
                initialize = false;
        }
    }

    /*
        if (CURRENT_SENSOR_INSTALLED)
        {
            Wire.begin();
            if (!powerSensor.begin())
            {
                ESP_LOGE(TAG, "# Could not connect to INA226 Current Sensor. Fix and Reboot");
                return false;
            }
            powerSensor.setMaxCurrentShunt(INA266_max_current, shunt);
            powerSensor.configure(shunt, current_LSB_mA, current_zero_offset_mA, bus_V_scaling_e4);
            // calibrate_ina226();
            ESP_LOGI(TAG, "# INA226 Current Sensor initialized.");
        }
            */
    if (CURRENT_SENSOR_INSTALLED && (currentSensor.getState() == SensorState::SENSOR_UNINITIALIZED))
    {
        if (!currentSensor.begin(CURRENT_SENSOR_PIN))
        {
            initialize = false;
        }
        currentSensor.setMovingAverage(32);
        currentSensor.setIIR(0.15f);

        currentSensor.calibrateZero();
    }

    if (TEMPERATURE_SENSOR_INSTALLED && (thermocoupleSensor.getState() == SensorState::SENSOR_UNINITIALIZED))
    {
        if (!thermocoupleSensor.begin(MAX31855_CS_PIN, -1, -1))
        {
            initialize = false;
        }
    }

    if (RPM_SENSOR_INSTALLED && (rpmSensor.getState() == SensorState::SENSOR_UNINITIALIZED))
    {
        RpmSensorConfig rpm_config;
        if (!rpmSensor.begin(RPM_SENSOR_PIN, rpm_config))
        {
            initialize = false;
        }
    }

    if (VOLTAGE_SENSOR_INSTALLED && (voltageSensor.getState() == SensorState::SENSOR_UNINITIALIZED))
    {
        if (!voltageSensor.begin(VOLTAGE_SENSOR_PIN))
        {
            initialize = false;
        }
    }

    ESP_LOGI(TAG, "Sensors %s initialized.", initialize ? "successfully" : "NOT");
    return initialize;
}

void ThrustStand::startAccumulation()
{
    ESP_LOGI(TAG, "startAccumulation...");
    resetCumulativeData();
    ESP_LOGI(TAG, "startAccumulation done");
    _accumulate_stats = true;
}

void ThrustStand::stopAccumulation()
{
    _accumulate_stats = false;
}

bool ThrustStand::updateSensors()
{
    bool ok = true;

    ok &= updateThrust();
    _state.thrustSensor = thrustSensor.getState();

    ok &= updateTorque();
    _state.torqueSensor = torqueSensor.getState();

    ok &= updateVoltage();
    _state.voltageSensor = voltageSensor.getState();

    ok &= updateCurrent();
    _state.currentSensor = currentSensor.getState();

    ok &= updateRPM();
    _state.rpmSensor = rpmSensor.getState();

    ok &= updateTemperature();
    _state.thermocoupleSensor = thermocoupleSensor.getState();

    return ok;
}

/**
 * @brief
 *
 */
void ThrustStand::tareSensors()
{
    thrustSensor.tare();
    torqueSensor.tare();

    if (CURRENT_SENSOR_INSTALLED)
    {
        currentSensor.calibrateZero();
    }
}

void ThrustStand::resetCumulativeData()
{
    ESP_LOGI(TAG, "resetCumulativeData");
    _cumDataSet = {};

    // Explicit baseline for max tracking
    _cumDataSet.values.temperature_max = -0.f;
}

bool ThrustStand::update()
{
    bool ok = true;

    if (hwEstop.isTriggered())
    {
        motor.emergencyStop("Hardware E-STOP pressed");
        setControlMode(ThrottleControlMode::MANUAL);
        // other actuators here

        ok = false;
    }
    // Run the motor control loop to handle acceleration/deceleration
    ok &= motor.update();
    _state.motor = motor.getState();
    _state.motorMountState = motor.getMountState();

    ok &= updateSensors();

    _statusLed.update();

    return ok;
}

void ThrustStand::setControlMode(ThrottleControlMode mode)
{
    _state.controlMode = mode;
}

float ThrustStand::setThrottle(float throttle, ThrottleSource source)
{

    return (setThrottle(throttle, true, 3000 * abs((motor.getCurrentThrottle() - throttle)) / 100.0f, source)); // Smooth acceleration based on throttle change
}

float ThrustStand::setThrottle(float throttle,
                               bool smooth,
                               unsigned long accelTimeMs,
                               ThrottleSource source)
{
    if (_state.controlMode == ThrottleControlMode::AUTOMATIC &&
        source == ThrottleSource::MANUAL)
    {
        ESP_LOGW(TAG, "Manual throttle ignored (test running)");
        return _currentDataSet.throttle; // or last commanded value
    }

    _currentDataSet.throttle = motor.setThrottle(throttle, smooth, accelTimeMs); // Smooth acceleration based on throttle change
    return (_currentDataSet.throttle);
}

/**
 * @brief returns the average values of the collected sensor data
 *
 * @return test_data_t
 */
test_data_t ThrustStand::getAverage() const
{
    test_data_t avg = {}; // zero-initialize

    // Throttle is not averaged – always take current value
    avg.throttle = _currentDataSet.throttle;

    /* ---------- Load cell / thrust related ---------- */

    if (_cumDataSet.samples.thrust > 0)
        avg.thrust = _cumDataSet.values.thrust / _cumDataSet.samples.thrust;
    else
        avg.thrust = _currentDataSet.thrust;

    if (_cumDataSet.samples.torque > 0)
        avg.torque = _cumDataSet.values.torque / _cumDataSet.samples.torque;
    else
        avg.torque = _currentDataSet.torque;

    if (_cumDataSet.samples.torque_cell_1 > 0)
        avg.torque_cell_1 =
            _cumDataSet.values.torque_cell_1 / _cumDataSet.samples.torque_cell_1;
    else
        avg.torque_cell_1 = _currentDataSet.torque_cell_1;

    if (_cumDataSet.samples.torque_cell_2 > 0)
        avg.torque_cell_2 =
            _cumDataSet.values.torque_cell_2 / _cumDataSet.samples.torque_cell_2;
    else
        avg.torque_cell_2 = _currentDataSet.torque_cell_2;

    /* ---------- Electrical ---------- */

    if (_cumDataSet.samples.voltage > 0)
        avg.voltage = _cumDataSet.values.voltage / _cumDataSet.samples.voltage;
    else
        avg.voltage = _currentDataSet.voltage;

    if (_cumDataSet.samples.current > 0)
        avg.current = _cumDataSet.values.current / _cumDataSet.samples.current;
    else
        avg.current = _currentDataSet.current;

    /* ---------- RPM ---------- */

    if (_cumDataSet.samples.rpm > 0)
        avg.rpm = _cumDataSet.values.rpm / _cumDataSet.samples.rpm;
    else
        avg.rpm = _currentDataSet.rpm;

    /* ---------- Temperature ---------- */

    if (_cumDataSet.samples.temperature > 0)
        avg.temperature =
            _cumDataSet.values.temperature / _cumDataSet.samples.temperature;
    else
        avg.temperature = _currentDataSet.temperature;

    // Max temperature is never averaged – it's a running max
    avg.temperature_max = _cumDataSet.values.temperature_max;

    return avg;
}

void ThrustStand::log_current_data_set()
{
    TorqueSensor::Calibration torque_cal = getTorqueCalibration();
    ESP_LOGI("log", "Throttle: %.1f%%  Thrust: %.3f  Torque: %.3f T-Load1: %.3f T-Load2: %.3f",
             _currentDataSet.throttle, _currentDataSet.thrust,
             _currentDataSet.torque, torque_cal.cal1, torque_cal.cal2);
    ESP_LOGI(TAG, " rpm: %.0f voltage: %.3f  current: %.3f   temp_max: %.1f",
             _currentDataSet.rpm, _currentDataSet.voltage, _currentDataSet.current, _currentDataSet.temperature_max);
    ;
}

float ThrustStand::getThrustCalFactor()
{
    return thrustSensor.getCalibration();
}

void ThrustStand::setThrustCalFactor(float factor)
{
    thrustSensor.setCalibration(factor);
    saveConfig();
}

void ThrustStand::setMaxThrottlePercent(float maxPercent)
{
    motor.setMaxThrottlePercent(maxPercent);
    saveConfig();
}

void ThrustStand::setTorqueCalibration(float cal1, float cal2, float distance)
{
    torqueSensor.setCalibration(cal1, cal2, distance);
    saveConfig();
}

TorqueSensor::Calibration ThrustStand::getTorqueCalibration()
{
    return (torqueSensor.getCalibration());
}

void ThrustStand::setCurrentSensitivity(float voltsPerAmp)
{
    currentSensor.setSensitivity(voltsPerAmp);
    saveConfig();
}

bool ThrustStand::setPulseRangeUs(uint16_t minPulseUs, uint16_t maxPulseUs)
{
    bool updateFlag = motor.setPulseRangeUs(minPulseUs, maxPulseUs);
    if (updateFlag)
        saveConfig();

    return updateFlag;
}

void ThrustStand::loadConfig()
{
    float interimValue;
    prefs.begin("thrust", false); // false=write mode in case the values as not yet set the will be written via putFloat, ....
    if (THRUST_SENSOR_INSTALLED)
    {
        if (!prefs.isKey("thrust_cal"))
            prefs.putFloat("thrust_cal", 1.0f);
        thrustSensor.setCalibration(prefs.getFloat("thrust_cal", 1.0f));
    }

    if (!prefs.isKey("thrust_max"))
        prefs.putFloat("thrust_max", 100.f);
    float thrust_max = prefs.getFloat("thrust_max", 100.f);
    motor.setMaxThrottlePercent(thrust_max);

    /* ---------- Torque ---------- */
    if (TORQUE_SENSOR_INSTALLED)
    {
        if (!prefs.isKey("torque_cal1"))
            prefs.putFloat("torque_cal1", 1.f);
        if (!prefs.isKey("torque_cal2"))
            prefs.putFloat("torque_cal2", 1.f);
        if (!prefs.isKey("torque_dist"))
            prefs.putFloat("torque_dist", 1.f);

        float cal1 = prefs.getFloat("torque_cal1", 1.f);
        float cal2 = prefs.getFloat("torque_cal2", 1.f);
        float dist = prefs.getFloat("torque_dist", 1.f);

        torqueSensor.setCalibration(cal1, cal2, dist);
    }
    /* ---------- Current ---------- */
    if (CURRENT_SENSOR_INSTALLED)
    {
        if (!prefs.isKey("current_sens"))
            prefs.putFloat("current_sens", 0.0264f);
        float current_sensitivity = prefs.getFloat("current_sens", 0.0264f);
        currentSensor.setSensitivity(current_sensitivity);
    }
    /* ---------- Voltage ---------- */
    if (!prefs.isKey("rTop"))
        prefs.putFloat("rTop", 10e3f);
    float rTop = prefs.getFloat("rTop", 10e3f);
    if (!prefs.isKey("rButtom"))
        prefs.putFloat("rButtom", 1e3f);
    float rButtom = prefs.getFloat("rButtom", 1e3f);
    voltageSensor.setResistorvalues(rTop, rButtom);

    /* ----------Motor ---------- */
    if (!prefs.isKey("minPulseUs"))
        prefs.putUInt("minPulseUs", 1000);
    if (!prefs.isKey("maxPulseUs"))
        prefs.putUInt("maxPulseUs", 2000);
    uint16_t minPulseUs = prefs.getUInt("minPulseUs");
    uint16_t maxPulseUs = prefs.getUInt("maxPulseUs");

    prefs.end();
}

void ThrustStand::saveConfig()
{
    /* ---------- Thrust ---------- */
    prefs.begin("thrust", false);
    if (THRUST_SENSOR_INSTALLED)
    {
        prefs.putFloat("thrust_cal", thrustSensor.getCalibration());
    }

    prefs.putFloat("thrust_max", getMaxThrottlePercent());
    /* ---------- Torque ---------- */
    if (TORQUE_SENSOR_INSTALLED)
    {
        auto torqueCal = torqueSensor.getCalibration();

        prefs.putFloat("torque_cal1", torqueCal.cal1);
        prefs.putFloat("torque_cal2", torqueCal.cal2);
        prefs.putFloat("torque_dist", torqueCal.distanceMM);
    }
    /* ---------- Current ---------- */
    if (CURRENT_SENSOR_INSTALLED)
    {
        prefs.putFloat("current_sens", currentSensor.getSensitivity());
    }
    /* ---------- Voltage ---------- */
    if (VOLTAGE_SENSOR_INSTALLED)
    {
        prefs.putFloat("rTop", voltageSensor.getTopResistor());
        prefs.putFloat("rBottom", voltageSensor.getBottomResistor());
    }

    /* ----------Motor ---------- */
    prefs.putUInt("minPulseUs", motor.getMinPulseUs());
    prefs.putUInt("maxPulseUs", motor.getMaxPulseUs());

    prefs.end();
}

void ThrustStand::resetNVS()
{
    nvs_flash_erase();
    nvs_flash_init();
}

bool ThrustStand::updateThrust()
{
    if (!thrustSensor.update())
        return false;

    _currentDataSet.thrust = thrustSensor.getThrust();

    if (_accumulate_stats)
    {
        _cumDataSet.values.thrust += _currentDataSet.thrust;
        _cumDataSet.samples.thrust++;
    }

    return true;
}

bool ThrustStand::updateCurrent()
{
    if (!currentSensor.update())
        return false;

    _currentDataSet.current = currentSensor.getCurrent_A();
    _currentDataSet.power =
        _currentDataSet.voltage * _currentDataSet.current;

    if (_accumulate_stats)
    {

        _cumDataSet.values.current += _currentDataSet.current;
        _cumDataSet.values.power += _currentDataSet.power;

        _cumDataSet.samples.current++;
        _cumDataSet.samples.power++;
    }

    return true;
}

bool ThrustStand::updateTemperature()
{
    if (!thermocoupleSensor.update())
        return false;

    float temp = thermocoupleSensor.getTemperatureInCelsius();
    _currentDataSet.temperature = temp;

    if (_accumulate_stats)
    {
        _cumDataSet.values.temperature += temp;
        _cumDataSet.samples.temperature++;

        if (temp > _cumDataSet.values.temperature_max)
            _cumDataSet.values.temperature_max = temp;
    }

    return true;
}

bool ThrustStand::updateRPM()
{
    if (!rpmSensor.update())
        return false;

    _currentDataSet.rpm = rpmSensor.getRPM();

    if (_accumulate_stats)
    {
        _cumDataSet.values.rpm += _currentDataSet.rpm;
        _cumDataSet.samples.rpm++;
    }

    return true;
}

bool ThrustStand::updateVoltage()
{
    if (!voltageSensor.update())
        return false;

    _currentDataSet.voltage = voltageSensor.getVoltage_V();

    if (_accumulate_stats)
    {
        _cumDataSet.values.voltage += _currentDataSet.voltage;
        _cumDataSet.samples.voltage++;
    }

    return true;
}

bool ThrustStand::updateTorque()
{
    if (!torqueSensor.update())
        return false;

    _currentDataSet.torque = torqueSensor.getTorqueNcm();
    _currentDataSet.torque_cell_1 = torqueSensor.getLoad1();
    _currentDataSet.torque_cell_2 = torqueSensor.getLoad2();

    if (_accumulate_stats)
    {
        _cumDataSet.values.torque += _currentDataSet.torque;
        _cumDataSet.samples.torque++;

        _cumDataSet.values.torque_cell_1 += _currentDataSet.torque_cell_1;
        _cumDataSet.samples.torque_cell_1++;

        _cumDataSet.values.torque_cell_2 += _currentDataSet.torque_cell_2;
        _cumDataSet.samples.torque_cell_2++;
    }

    return true;
}

void ThrustStand::updateStatusLed()
{
    const StatusLed::State newState = deriveLedState();

    _statusLed.setState(newState);
    _statusLed.update(); // timing / blinking handled inside LED class
}

StatusLed::State ThrustStand::deriveLedState() const
{
    // 1 Hard errors first (highest priority)
    if (hwEstop.isTriggered())
        return StatusLed::State::ERROR;

    if (_state.controlMode == ThrottleControlMode::AUTOMATIC)
        return StatusLed::State::RUNNING;

    /*    if (hasAnyCriticalError())
        return StatusLed::State::ERROR;

    if (_webHealth == SubsystemHealth::ERROR)
        return StatusLed::State::WARNING; // yellow blinking

    if (state.controlMode == ThrottleControlMode::AUTOMATIC)
        return StatusLed::State::RUNNING;

    if (!isReady())
        return StatusLed::State::BUSY;
    */
    // 4 Normal idle
    return StatusLed::State::READY;
}