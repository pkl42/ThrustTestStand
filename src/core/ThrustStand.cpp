/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <Arduino.h>
#include <esp_log.h>

#include "core/Config.h"
#include <Preferences.h>
#include <nvs_flash.h> // deleting nvs flash
#include <cfloat>
#include "core/ThrustStand.h"

const char *ThrustStand::TAG = "ThrustStand";

static Preferences prefs;

test_data_t ThrustStand::_currentDataSet = {};
test_data_accu_t ThrustStand::_accuDataSet = {};

ThrustStand::ThrustStand()
    : _thrustSensor(HX711_DOUT_1_PIN, HX711_SCK_1_PIN),
      _torqueSensor(HX711_DOUT_2_PIN, HX711_SCK_2_PIN,
                    HX711_DOUT_3_PIN, HX711_SCK_3_PIN),
      _rpmSensor(),
      _tempSensor(),
      _currentSensor(),
      _voltageSensor(),
      _hwEstop(ESTOP_PIN),
      _pwmDriver(new PwmEscDriver(MOTOR_ESC_PIN, 50, 13, 1)),
      _motor(_pwmDriver) // pass driver directly

{
    // other members can be initialized here
    _escDriverType = EscDriverType::ESC_DRIVER_PWM;
}

ThrustStand::~ThrustStand()
{
    _motor.stop(); // ensure motor is stopped
    delete _pwmDriver;
    delete _dshotDriver;
}

bool ThrustStand::switchDriver(EscDriverType type)
{
    // No-op if already active
    if (_escDriverType == type)
        return true;

    // Always stop motor before switching signal protocol
    _motor.stop();

    EscSignalDriver *newDriver = nullptr;

    switch (type)
    {
    case EscDriverType::ESC_DRIVER_PWM:
    {
        if (!_pwmDriver)
        {
            _pwmDriver = new PwmEscDriver(
                MOTOR_ESC_PIN,
                50, // Hz
                13, // resolution
                1   // LEDC channel
            );
        }

        newDriver = _pwmDriver;
        break;
    }

    case EscDriverType::ESC_DRIVER_DSHOT150:
    case EscDriverType::ESC_DRIVER_DSHOT300:
    case EscDriverType::ESC_DRIVER_DSHOT600:
    {
        if (!_dshotDriver)
        {
            _dshotDriver = new DShotEscDriver(
                MOTOR_ESC_PIN,
                DShotEscDriver::DShotRate::DSHOT300 // temporary default
            );
        }

        // Map EscDriverType → DShotRate
        DShotEscDriver::DShotRate rate;
        switch (type)
        {
        case EscDriverType::ESC_DRIVER_DSHOT150:
            rate = DShotEscDriver::DShotRate::DSHOT150;
            break;
        case EscDriverType::ESC_DRIVER_DSHOT600:
            rate = DShotEscDriver::DShotRate::DSHOT600;
            break;
        case EscDriverType::ESC_DRIVER_DSHOT300:
        default:
            rate = DShotEscDriver::DShotRate::DSHOT300;
            break;
        }

        if (!_dshotDriver->setRate(rate))
        {
            ESP_LOGE(TAG, "Failed to set DShot rate");
            return false;
        }

        newDriver = _dshotDriver;
        break;
    }

    case EscDriverType::ESC_DRIVER_NONE:
    default:
        ESP_LOGW(TAG, "ESC driver NONE selected");
        _escDriverType = EscDriverType::ESC_DRIVER_NONE;
        return true;
    }

    if (!newDriver)
        return false;

    // Bind new driver to MotorESC
    _motor.setDriver(newDriver);

    // Initialize hardware for new protocol
    if (!_motor.begin())
    {
        ESP_LOGE(TAG, "MotorESC begin() failed after driver switch");
        return false;
    }

    _escDriverType = type;
    saveConfig();

    ESP_LOGI(TAG, "ESC driver switched to %s", escDriverToString(type));
    return true;
}

bool ThrustStand::init()
{

    loadConfig();

    pinMode(CAGE_SWITCH_PIN, INPUT);
    ESP_LOGI(TAG, "Cage Close Switch assigned on GPIO: %i", CAGE_SWITCH_PIN);

    if (!init_sensors())
    {

        return false;
    }

    if (!switchDriver(_escDriverType))
    {

        return false;
    }

    if (!_motor.begin())
    {

        return false;
    };

    _hwEstop.attachActuator(&_motor);
    _hwEstop.begin(); // attach interrupts, ready to stop motor

    return true;
}

bool ThrustStand::armMotor()
{
    bool armFlag = false;
    // --- Fail-safe: check E-Stop before arming ---

    if (isSafeToArm())
    {
        _state.idleTimeMs = millis();
        armFlag = _motor.arm(); // Arm the ESC
    }
    else
    {
        ESP_LOGW(TAG, "Hardware E-Stop/Cage open active: motor NOT armed");
    }

    return armFlag;
}

bool ThrustStand::disarmMotor()
{
    return _motor.disarm(); // Disarm the ESC
}

bool ThrustStand::init_sensors(bool force)
{
    bool initialize = true;
    ESP_LOGI(TAG, "Initializing Sensors%s ...",
             force ? " (forced)" : "");

    // pre-check, thrust stand is in idle mode

    if ((_state.controlMode == ThrottleControlMode::AUTOMATIC) || (_motor.getCurrentThrottle() > 0.f))
    {
        ESP_LOGW(TAG, "Initializing Sensors not allowed as system is not idle.");
        return false;
    }

    if (THRUST_SENSOR_INSTALLED && (_thrustSensor.getState() == SensorState::SENSOR_UNINITIALIZED))
    {
        if (!_thrustSensor.begin()) // calibration value already set by loadConfig()
            initialize = false;
    }

    if (TORQUE_SENSOR_INSTALLED)
    {
        if (_torqueSensor.getState() != SensorState::SENSOR_READY)
        {
            if (!_torqueSensor.begin()) // calibration value already set by loadConfig()
                initialize = false;
        }
    }

    /*
        if (CURRENT_SENSOR_INSTALLED && (_currentSensor.getState() == SensorState::SENSOR_UNINITIALIZED))
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
    if (CURRENT_SENSOR_INSTALLED && (_currentSensor.getState() == SensorState::SENSOR_UNINITIALIZED) || force)
    {
        if (!_currentSensor.begin(CURRENT_SENSOR_PIN))
        {
            initialize = false;
        }
        _currentSensor.setMovingAverage(32);
        _currentSensor.setIIR(0.15f);

        _currentSensor.tare();
    }

    if ((TEMPERATURE_SENSOR_TYPE > TEMP_SENSOR_NONE) && (_tempSensor.getState() == SensorState::SENSOR_UNINITIALIZED) || force)
    {

        if (!_tempSensor.begin())
        {
            initialize = false;
        }
    }

    if (RPM_SENSOR_INSTALLED && ((_rpmSensor.getState() == SensorState::SENSOR_UNINITIALIZED || force)))
    {
        RpmSensorConfig rpm_config;
        if (!_rpmSensor.begin(RPM_SENSOR_PIN, rpm_config))
        {
            initialize = false;
        }
    }

    if (VOLTAGE_SENSOR_INSTALLED && (_voltageSensor.getState() == SensorState::SENSOR_UNINITIALIZED))
    {
        if (!_voltageSensor.begin(VOLTAGE_SENSOR_PIN))
        {
            initialize = false;
        }
    }

    ESP_LOGI(TAG, "Sensors %s initialized.", initialize ? "successfully" : "NOT");
    return initialize;
}

void ThrustStand::startAccumulation()
{
    ESP_LOGI(TAG, "startAccumulation");
    resetAccumulativeData();
    _accuDataSet.t_ms = millis();
    _accumulate_stats = true;
}

void ThrustStand::stopAccumulation()
{
    // ESP_LOGI(TAG, "stopAccumulation");
    _accumulate_stats = false;
}

bool ThrustStand::updateSensors()
{
    bool ok = true;

    ok &= updateThrust();
    _state.thrustSensor = _thrustSensor.getState();

    ok &= updateTorque();
    _state.torqueSensor = _torqueSensor.getState();

    ok &= updateVoltage();
    _state.voltageSensor = _voltageSensor.getState();

    ok &= updateCurrent();
    _state.currentSensor = _currentSensor.getState();

    ok &= updateRPM();
    _state.rpmSensor = _rpmSensor.getState();

    ok &= updateTemperature();
    _state.tempSensor = _tempSensor.getState();

    ok &= updateDerivedSensorData();

    ok &= updateThottle();

    return ok;
}

bool ThrustStand::tareSensors()
{
    bool success = true;

    success &= _thrustSensor.tare();
    success &= _torqueSensor.tare();

    if (CURRENT_SENSOR_INSTALLED)
    {
        success &= _currentSensor.tare();
    }

    return success;
}

void ThrustStand::resetAccumulativeData()
{
    ESP_LOGI(TAG, "resetAccumulativeData");
    _accuDataSet = {};
}

bool ThrustStand::update()
{
    bool ok = true;
    // 1. Hardware E-STOP (highest priority)
    if (_hwEstop.isTriggered())
    {
        _motor.emergencyStop("Hardware E-STOP pressed");
        setControlMode(ThrottleControlMode::MANUAL);
        // other actuators here

        ok = false;
    }
    // 2. Run the motor control loop to handle acceleration/deceleration
    ok &= _motor.update();
    _state.motor = _motor.getState();
    _state.motorMountState = _motor.getMountState();
    // 3. Sensor update
    ok &= updateSensors();

    // 4. Software safety supervision (after sensors!)

    if (!checkSafetyLimits())
    {
        ok = false;
    }
    // 5. Idle Time -> AutoDisarm
    updateIdleActivity();
    checkAutoDisarmOnIdle();

    // 6. Status indication
    _statusLed.update();

    return ok;
}

void ThrustStand::updateIdleActivity()
{
    const uint32_t nowMs = millis();

    const bool standActive =
        (_state.controlMode == ThrottleControlMode::AUTOMATIC) ||
        (_state.controlMode == ThrottleControlMode::MANUAL &&
         _motor.getCurrentThrottle() > 0.0f);

    if (standActive)
    {
        _state.idleTimeMs = nowMs;
    }
}

void ThrustStand::checkAutoDisarmOnIdle()
{
    if (_state.controlMode != ThrottleControlMode::MANUAL)
        return;

    if (!(_state.motorMountState == MotorESC::EscState::ARMED))
        return;

    if (_motor.getCurrentThrottle() > 0.0f)
        return;

    const uint32_t nowMs = millis();
    const uint32_t idleMs = nowMs - _state.idleTimeMs;

    if (idleMs >= _autoDisarmMs)
    {
        ESP_LOGW(TAG,
                 "Auto-disarming motor after %lu ms idle in MANUAL mode",
                 idleMs);

        _motor.disarm();
    }
}

void ThrustStand::setControlMode(ThrottleControlMode mode)
{
    if (_state.controlMode == mode)
        return;

    if (mode == ThrottleControlMode::AUTOMATIC)
    {
        // Entering test mode
        _safety.testLimits = _safety.coreLimits;
    }
    else if (_state.controlMode == ThrottleControlMode::AUTOMATIC)
    {
        // Leaving test mode
        _safety.testLimits = {};
        _state.idleTimeMs = millis();
    }

    _state.controlMode = mode;
}

float ThrustStand::setThrottle(float throttle, ThrottleSource source)
{

    return (setThrottle(throttle, true, 3000 * abs((_motor.getCurrentThrottle() - throttle)) / 100.0f, source)); // Smooth acceleration based on throttle change
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
        return _currentDataSet.throttle_act; // or last commanded value
    }

    float maxThrottle =
        (source == ThrottleSource::MANUAL)
            ? _safety.coreLimits.maxThrottlePercent
            : 100.f;

    throttle = constrain(throttle, 0.f, maxThrottle);
    _currentDataSet.throttle_cmd = _motor.setThrottle(throttle, smooth, accelTimeMs); // Smooth acceleration based on throttle change

    return (_currentDataSet.throttle_cmd);
}

float ThrustStand::getThrustCalFactor()
{
    return _thrustSensor.getCalibration();
}

void ThrustStand::setThrustCalFactor(float factor)
{
    _thrustSensor.setCalibration(factor);
    saveConfig();
}

void ThrustStand::setTorqueCalibration(float cal1, float cal2, float distance)
{
    _torqueSensor.setCalibration(cal1, cal2, distance);
    saveConfig();
}

TorqueSensor::Calibration ThrustStand::getTorqueCalibration()
{
    return (_torqueSensor.getCalibration());
}

void ThrustStand::setCurrentSensitivity(float voltsPerAmp)
{
    _currentSensor.setSensitivity(voltsPerAmp);
    saveConfig();
}

bool ThrustStand::setPulseRangeUs(uint16_t minPulseUs, uint16_t maxPulseUs)
{
    bool updateFlag = _motor.setPulseRangeUs(minPulseUs, maxPulseUs);
    if (updateFlag)
        saveConfig();

    return updateFlag;
}

bool ThrustStand::setVoltageCalibrationFactor(float calibrationFactor)
{
    bool updateFlag = _voltageSensor.setCalibrationFactor(calibrationFactor);
    if (updateFlag)
        saveConfig();

    return updateFlag;
}

void ThrustStand::autoCalibrateCurrentSensor(float actualCurrent_A, float measuredCurrent_A)
{
    _currentSensor.calibrateSensitivityByCurrent(actualCurrent_A, measuredCurrent_A);
    saveConfig();
};

void ThrustStand::setAutoDisarmTimeOut(uint32_t autoDisarmS)
{
    autoDisarmS = constrain(autoDisarmS, 5, 3600);
    _autoDisarmMs = autoDisarmS * 1000UL;
    saveConfig();
}

// Safety Limits

void ThrustStand::setThrottleLimitPercent(float percent, SafetyTripSource source)
{

    if (source == SafetyTripSource::CORE_LIMIT)
    {
        _safety.coreLimits.maxThrottlePercent = constrain(percent, 0.0f, 100.0f);
        saveConfig();
    }
    if (source == SafetyTripSource::TEST_LIMIT)
    {
        if (percent < _safety.coreLimits.maxThrottlePercent)
        {
            _safety.testLimits.maxThrottlePercent = percent;
        }
    }
}

void ThrustStand::setBatteryPreset(BatteryPreset preset,
                                   SafetyTripSource source)
{
    if (source == SafetyTripSource::CORE_LIMIT)
    {
        _safety.coreLimits.batteryPreset = preset;
        saveConfig();
    }
    // not sure if it make sense to purely set the battery preset on TEST_LIMIT
    // but for keeping it aligned with the other limit settings
    if (source == SafetyTripSource::TEST_LIMIT)
    {
        _safety.testLimits.batteryPreset = preset;
    }
}

void ThrustStand::setCurrentLimitA(float maxCurrentA, SafetyTripSource source)
{
    if (maxCurrentA > CURRENT_SENSOR_MAX)
        return;

    if (source == SafetyTripSource::CORE_LIMIT)
    {
        _safety.coreLimits.maxCurrentA = maxCurrentA;
        saveConfig();
    }
    if (source == SafetyTripSource::TEST_LIMIT)
    {
        if (maxCurrentA < _safety.coreLimits.maxCurrentA)
        {
            _safety.testLimits.maxCurrentA = maxCurrentA;
        }
    }
}

void ThrustStand::setVoltageLimitMaxV(float maxVoltageV, SafetyTripSource source)
{
    if (maxVoltageV > VOLTAGE_SENSOR_MAX)
        return;

    if (source == SafetyTripSource::CORE_LIMIT)
    {
        _safety.coreLimits.maxVoltageV = maxVoltageV;
        saveConfig();
    }

    if (source == SafetyTripSource::TEST_LIMIT)
    {
        if (maxVoltageV <= _safety.coreLimits.maxVoltageV)
        {
            _safety.testLimits.maxVoltageV = maxVoltageV;
        }
    }
}

void ThrustStand::setVoltageLimitMinV(float minVoltageV, SafetyTripSource source)
{
    if (source == SafetyTripSource::CORE_LIMIT)
    {
        if (minVoltageV < _safety.coreLimits.maxVoltageV)
        {
            _safety.coreLimits.minVoltageV = minVoltageV;
            saveConfig();
        }
    }

    if (source == SafetyTripSource::TEST_LIMIT)
    {
        // Must remain inside the hardware-safe envelope
        if (minVoltageV <= _safety.coreLimits.minVoltageV)
            return;

        // Must remain consistent with test max
        if (minVoltageV >= _safety.testLimits.maxVoltageV)
            return;

        _safety.testLimits.minVoltageV = minVoltageV;
    }
}

void ThrustStand::setThrustLimitGF(float maxThrustGF, SafetyTripSource source)
{
    if (maxThrustGF > THRUST_SENSOR_MAX)
        return;

    if (source == SafetyTripSource::CORE_LIMIT)
    {
        _safety.coreLimits.maxThrustGF = maxThrustGF;
        saveConfig();
    }
    if (source == SafetyTripSource::TEST_LIMIT)
    {
        if (maxThrustGF < _safety.coreLimits.maxThrustGF)
        {
            _safety.testLimits.maxThrustGF = maxThrustGF;
        }
    }
}

void ThrustStand::setTemperatureLimitC(float maxTemperatureC, SafetyTripSource source)
{
    if (maxTemperatureC > TEMPERATURE_SENSOR_MAX)
        return;

    if (source == SafetyTripSource::CORE_LIMIT)
    {
        _safety.coreLimits.maxTemperatureC = maxTemperatureC;
        saveConfig();
    }
    if (source == SafetyTripSource::TEST_LIMIT)
    {
        if (maxTemperatureC < _safety.coreLimits.maxThrustGF)
        {
            _safety.testLimits.maxTemperatureC = maxTemperatureC;
        }
    }
}

bool ThrustStand::checkSafetyLimits()
{
    // Already tripped → keep enforcing safe state
    if (_safety.state.tripped)
        return false;

    if (_hwEstop.isTriggered())
    {
        triggerSafetyTrip(SafetyTripSource::CORE_LIMIT, SafetyTripReason::SAFETY_TRIP_ESTOP, 0.f);
        return false;
    }

    if (isCageOpen())
    { // NC switch open → 0V

        triggerSafetyTrip(SafetyTripSource::CORE_LIMIT, SafetyTripReason::SAFETY_TRIP_PROP_CAGE_OPEN, 0.f);
        return false;
    }

    if (_currentDataSet.throttle_act > _safety.coreLimits.maxThrottlePercent)
    {

        triggerSafetyTrip(SafetyTripSource::CORE_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_THROTTLE, _currentDataSet.throttle_act);
        return false;
    }

    // Over-current
    if (_safety.coreLimits.maxCurrentA > 0.0f &&
        _currentDataSet.current > _safety.coreLimits.maxCurrentA)
    {
        triggerSafetyTrip(SafetyTripSource::CORE_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_CURRENT, _currentDataSet.current);
        return false;
    }

    // Over-voltage
    if (_safety.coreLimits.maxVoltageV > 0.0f &&
        _currentDataSet.voltage > _safety.coreLimits.maxVoltageV)
    {
        triggerSafetyTrip(SafetyTripSource::CORE_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_VOLTAGE, _currentDataSet.voltage);
        return false;
    }

    // Under-voltage
    if (_currentDataSet.voltage < _safety.coreLimits.minVoltageV)
    {
        triggerSafetyTrip(SafetyTripSource::CORE_LIMIT, SafetyTripReason::SAFETY_TRIP_UNDER_VOLTAGE, _currentDataSet.voltage);
        return false;
    }

    // Over-thrust
    if (_safety.coreLimits.maxThrustGF > 0.0f &&
        _currentDataSet.thrust > _safety.coreLimits.maxThrustGF)
    {
        triggerSafetyTrip(SafetyTripSource::CORE_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_THRUST, _currentDataSet.thrust);
        return false;
    }

    // Over-Temperature
    if (_currentDataSet.temperature > _safety.coreLimits.maxTemperatureC)
    {
        triggerSafetyTrip(SafetyTripSource::CORE_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_TEMPERATURE, _currentDataSet.temperature);
        return false;
    }

    // Over-RPM not implemented
    // if (_currentDataSet.rpm > _safety.coreLimits.maxRPM)
    // {
    //     triggerSafetyTrip(SafetyTripSource::CORE_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_RPM, _currentDataSet.rpm);
    //     return false;
    // }

    if (_state.controlMode == ThrottleControlMode::MANUAL)
        return true;

    // Additional Safety check during test

    if (_currentDataSet.throttle_act > _safety.testLimits.maxThrottlePercent)
    {

        triggerSafetyTrip(SafetyTripSource::TEST_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_THROTTLE, _currentDataSet.throttle_act);
        return false;
    }

    // Over-current
    if (_safety.testLimits.maxCurrentA > 0.0f &&
        _currentDataSet.current > _safety.testLimits.maxCurrentA)
    {
        triggerSafetyTrip(SafetyTripSource::TEST_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_CURRENT, _currentDataSet.current);
        return false;
    }

    // Over-voltage
    if (_safety.testLimits.maxVoltageV > 0.0f &&
        _currentDataSet.voltage > _safety.testLimits.maxVoltageV)
    {
        triggerSafetyTrip(SafetyTripSource::TEST_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_VOLTAGE, _currentDataSet.voltage);
        return false;
    }

    // Under-voltage
    if (_currentDataSet.voltage < _safety.testLimits.minVoltageV)
    {
        triggerSafetyTrip(SafetyTripSource::TEST_LIMIT, SafetyTripReason::SAFETY_TRIP_UNDER_VOLTAGE, _currentDataSet.voltage);
        return false;
    }

    // Over-thrust
    if (_safety.testLimits.maxThrustGF > 0.0f &&
        _currentDataSet.thrust > _safety.testLimits.maxThrustGF)
    {
        triggerSafetyTrip(SafetyTripSource::TEST_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_THRUST, _currentDataSet.thrust);
        return false;
    }

    if (_currentDataSet.temperature > _safety.testLimits.maxTemperatureC)
    {
        triggerSafetyTrip(SafetyTripSource::TEST_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_TEMPERATURE, _currentDataSet.temperature);
        return false;
    }

    // Over-RPM not implemented
    // if (_currentDataSet.rpm > _safety.testLimits.maxRPM)
    // {
    //     triggerSafetyTrip(SafetyTripSource::TEST_LIMIT, SafetyTripReason::SAFETY_TRIP_OVER_RPM, _currentDataSet.rpm);
    //     return false;
    // }

    return true;
}

ActuatorState ThrustStand::mapTripToState(SafetyTripReason reason)
{
    switch (reason)
    {
    case SafetyTripReason::SAFETY_TRIP_NONE:
        return ActuatorState::ACTU_READY;

    case SafetyTripReason::SAFETY_TRIP_ESTOP:
    case SafetyTripReason::SAFETY_TRIP_PROP_CAGE_OPEN:
        return ActuatorState::ACTU_E_STOP;

    case SafetyTripReason::SAFETY_TRIP_USER_ABORT:
        return ActuatorState::ACTU_STOPPED;

    default:
        return ActuatorState::ACTU_ERROR;
    }
}

void ThrustStand::triggerSafetyTrip(SafetyTripSource source, SafetyTripReason reason, float value)
{
    if (reason == SafetyTripReason::SAFETY_TRIP_NONE)
        return;

    if (_safety.state.tripped)
        return;

    _safety.state.tripped = true;
    _safety.state.reason = reason;
    _safety.state.tripValue = value;
    _safety.state.source = source;

    if (_safety.state.source == SafetyTripSource::TEST_LIMIT)
    {
        _motor.setThrottle(0., true);
        setControlMode(ThrottleControlMode::MANUAL);
        return;
    }

    ActuatorState stopSeverity = mapTripToState(reason);

    switch (stopSeverity)
    {
    case ActuatorState::ACTU_ERROR:
        _motor.stopWithError();
        break;

    case ActuatorState::ACTU_STOPPED:
        _motor.stop();
        break;

    case ActuatorState::ACTU_E_STOP:
        _motor.emergencyStop("Safety limit violated");
        break;

    default:
        // Should never happen — fail safe
        _motor.stopWithError();
        break;
    }

    setControlMode(ThrottleControlMode::MANUAL);
}

void ThrustStand::clearSafetyTrip()
{
    if (!_hwEstop.reset())
    {
        ESP_LOGW(TAG, "E-STOP still physically active - cannot clear");
        return;
    }

    if (!isSafeToArm())
    {
        ESP_LOGI(TAG, "isSafeToArm is set to false - not clearance");
        return;
    }

    _safety.state.tripped = false;
    _safety.state.reason = SafetyTripReason::SAFETY_TRIP_NONE;
    _safety.state.tripValue = 0.0f;
    _safety.state.source = SafetyTripSource::NONE;
    _motor.resetEmergencyStop();
    _motor.resetError();
}

bool ThrustStand::isSafeToArm() const
{
    return !_hwEstop.isTriggered() && !isCageOpen();
}

bool ThrustStand::isCageOpen() const
{
    return digitalRead(CAGE_SWITCH_PIN) == LOW; // LOW = open/unsafe
}

void ThrustStand::loadConfig()
{
    float interimValue;
    prefs.begin("thrust", false); // false=write mode in case the values as not yet set the will be written via putFloat, ....
    if (THRUST_SENSOR_INSTALLED)
    {
        if (!prefs.isKey("thrust_cal"))
            prefs.putFloat("thrust_cal", 1.0f);
        _thrustSensor.setCalibration(prefs.getFloat("thrust_cal", 1.0f));
    }

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

        _torqueSensor.setCalibration(cal1, cal2, dist);
    }
    /* ---------- Current ---------- */
    if (CURRENT_SENSOR_INSTALLED)
    {
        if (!prefs.isKey("current_sens"))
            prefs.putFloat("current_sens", 0.0264f);
        float current_sensitivity = prefs.getFloat("current_sens", 0.0264f);
        _currentSensor.setSensitivity(current_sensitivity);
    }
    /* ---------- Voltage ---------- */
    if (VOLTAGE_SENSOR_INSTALLED)
    {
        if (!prefs.isKey("rTop"))
            prefs.putFloat("rTop", 10e3f);
        float rTop = prefs.getFloat("rTop", 10e3f);
        if (!prefs.isKey("rButtom"))
            prefs.putFloat("rButtom", 1e3f);
        float rButtom = prefs.getFloat("rButtom", 1e3f);
        _voltageSensor.setResistorvalues(rTop, rButtom);

        if (!prefs.isKey("volt_calib"))
            prefs.putFloat("volt_calib", 1.f);
        float volt_calib = prefs.getFloat("volt_calib", 1.);
        _voltageSensor.setCalibrationFactor(volt_calib);
    }

    /* ----------Motor ---------- */
    if (!prefs.isKey("minPulseUs"))
        prefs.putUInt("minPulseUs", 1000);
    if (!prefs.isKey("maxPulseUs"))
        prefs.putUInt("maxPulseUs", 2000);
    uint16_t minPulseUs = prefs.getUInt("minPulseUs");
    uint16_t maxPulseUs = prefs.getUInt("maxPulseUs");
    _motor.setPulseRangeUs(minPulseUs, maxPulseUs);

    if (!prefs.isKey("autoDisarmMs"))
        prefs.putUInt("autoDisarmMs", _autoDisarmMs);
    _autoDisarmMs = prefs.getUInt("autoDisarmMs");

    if (!prefs.isKey("escDriverType"))
        prefs.putUInt("escDriverType", static_cast<uint8_t>(_escDriverType));
    _escDriverType = static_cast<EscDriverType>(prefs.getUInt("escDriverType"));

    /* ---------- Safety ---------- */
    if (!prefs.isKey("maxCurrentA"))
        prefs.putFloat("maxCurrentA", CURRENT_SENSOR_MAX);
    _safety.coreLimits.maxCurrentA = prefs.getFloat("maxCurrentA");

    if (!prefs.isKey("maxThrustGF"))
        prefs.putFloat("maxThrustGF", THRUST_SENSOR_MAX);
    _safety.coreLimits.maxThrustGF = prefs.getFloat("maxThrustGF");

    if (!prefs.isKey("maxThrottle"))
        prefs.putFloat("maxThrottle", 100.f);
    _safety.coreLimits.maxThrottlePercent = prefs.getFloat("maxThrottle");

    if (!prefs.isKey("maxVoltageV"))
        prefs.putFloat("maxVoltageV", VOLTAGE_SENSOR_MAX);
    _safety.coreLimits.maxVoltageV = prefs.getFloat("maxVoltageV");

    if (!prefs.isKey("minVoltageV"))
        prefs.putFloat("minVoltageV", 0.f);
    _safety.coreLimits.minVoltageV = prefs.getFloat("minVoltageV");

    if (!prefs.isKey("batteryPreset"))
    {
        prefs.putUInt("batteryPreset", batteryPresetToCells(BatteryPreset::BATTERY_PRESET_NONE));
    }
    uint8_t batteryPreset = prefs.getUInt("batteryPreset");
    _safety.coreLimits.batteryPreset = cellsToPreset(batteryPreset);

    prefs.end();
}

void ThrustStand::saveConfig()
{
    /* ---------- Thrust ---------- */
    prefs.begin("thrust", false);
    if (THRUST_SENSOR_INSTALLED)
    {
        prefs.putFloat("thrust_cal", _thrustSensor.getCalibration());
    }

    /* ---------- Torque ---------- */
    if (TORQUE_SENSOR_INSTALLED)
    {
        auto torqueCal = _torqueSensor.getCalibration();

        prefs.putFloat("torque_cal1", torqueCal.cal1);
        prefs.putFloat("torque_cal2", torqueCal.cal2);
        prefs.putFloat("torque_dist", torqueCal.distanceMM);
    }
    /* ---------- Current ---------- */
    if (CURRENT_SENSOR_INSTALLED)
    {
        prefs.putFloat("current_sens", _currentSensor.getSensitivity());
    }
    /* ---------- Voltage ---------- */
    if (VOLTAGE_SENSOR_INSTALLED)
    {
        prefs.putFloat("rTop", _voltageSensor.getTopResistor());
        prefs.putFloat("rBottom", _voltageSensor.getBottomResistor());
        prefs.putFloat("volt_calib", _voltageSensor.getCalibrationFactor());
    }

    /* ----------Motor ---------- */
    prefs.putUInt("minPulseUs", _motor.getMinPulseUs());
    prefs.putUInt("maxPulseUs", _motor.getMaxPulseUs());

    prefs.putUInt("escDriverType", static_cast<uint8_t>(_escDriverType));

    /* ---------Security Limits ---------- */
    prefs.putFloat("maxCurrentA", _safety.coreLimits.maxCurrentA);
    prefs.putFloat("maxThrustGF", _safety.coreLimits.maxThrustGF);
    prefs.putFloat("maxThrottle", _safety.coreLimits.maxThrottlePercent);
    prefs.putFloat("maxVoltageV", _safety.coreLimits.maxVoltageV);
    prefs.putFloat("minVoltageV", _safety.coreLimits.minVoltageV);
    prefs.putFloat("maxTempC", _safety.coreLimits.maxTemperatureC);

    prefs.putUInt("autoDisarmMs", _autoDisarmMs);

    prefs.putUInt("batteryPreset", batteryPresetToCells(_safety.coreLimits.batteryPreset));

    prefs.end();
}

void ThrustStand::resetNVS()
{
    nvs_flash_erase();
    nvs_flash_init();
}

static inline void updateStats(sensor_stats_t &s, float x)
{
    if (s.n == 0)
    {
        s.mean = x;
        s.M2 = 0.0f;
        s.min = x;
        s.max = x;
        s.n = 1;
        return;
    }

    s.n++;

    float delta = x - s.mean;
    s.mean += delta / s.n;
    float delta2 = x - s.mean;
    s.M2 += delta * delta2;

    if (x < s.min)
        s.min = x;
    if (x > s.max)
        s.max = x;
}

bool ThrustStand::updateThottle()
{
    _currentDataSet.throttle_act = _motor.getCurrentThrottle();

    if (_accumulate_stats)
    {
        updateStats(_accuDataSet.throttle, _currentDataSet.throttle_act);
    }

    return true;
}

bool ThrustStand::updateThrust()
{
    if (!_thrustSensor.update())
        return false;

    _currentDataSet.thrust = _thrustSensor.getThrust();

    if (_accumulate_stats)
    {
        updateStats(_accuDataSet.thrust, _currentDataSet.thrust);
    }

    return true;
}

bool ThrustStand::updateCurrent()
{
    if (!_currentSensor.update())
        return false;

    _currentDataSet.current = _currentSensor.getCurrent_A();

    if (_accumulate_stats)
    {
        updateStats(_accuDataSet.current, _currentDataSet.current);
    }

    return true;
}

bool ThrustStand::updateTemperature()
{
    if (!_tempSensor.update())
        return false;

    float temp = _tempSensor.getTemperatureInCelsius();
    _currentDataSet.temperature = temp;

    if (_accumulate_stats)
    {
        updateStats(_accuDataSet.temperature, temp);
    }

    return true;
}

bool ThrustStand::updateRPM()
{
    if (!_rpmSensor.update())
        return false;

    _currentDataSet.rpm = _rpmSensor.getRPM();

    if (_accumulate_stats)
    {
        updateStats(_accuDataSet.rpm, _currentDataSet.rpm);
    }

    return true;
}

bool ThrustStand::updateVoltage()
{
    if (!_voltageSensor.update())
        return false;

    _currentDataSet.voltage = _voltageSensor.getVoltage_V();

    if (_accumulate_stats)
    {
        updateStats(_accuDataSet.voltage, _currentDataSet.voltage);
    }

    return true;
}

bool ThrustStand::updateTorque()
{
    if (!_torqueSensor.update())
        return false;

    _currentDataSet.torque = _torqueSensor.getTorqueNcm();
    _currentDataSet.torque_cell_1 = _torqueSensor.getLoad1();
    _currentDataSet.torque_cell_2 = _torqueSensor.getLoad2();

    if (_accumulate_stats)
    {
        updateStats(_accuDataSet.torque, _currentDataSet.torque);
        updateStats(_accuDataSet.torque_cell_1, _currentDataSet.torque_cell_1);
        updateStats(_accuDataSet.torque_cell_2, _currentDataSet.torque_cell_2);
    }

    return true;
}

bool ThrustStand::updateDerivedSensorData()
{

    _currentDataSet.power = _currentDataSet.voltage * _currentDataSet.current;
    _currentDataSet.thrust_ratio = (_currentDataSet.power > 0.0001f) ? (_currentDataSet.thrust / _currentDataSet.power) : 0.0f;

    if (_accumulate_stats)
    {
        updateStats(_accuDataSet.power, _currentDataSet.power);
        updateStats(_accuDataSet.thrust_ratio, _currentDataSet.thrust_ratio);
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
    if (_hwEstop.isTriggered())
        return StatusLed::State::ERROR;

    if (_state.controlMode == ThrottleControlMode::AUTOMATIC)
        return StatusLed::State::RUNNING;

    /*    if (hasAnyCriticalError())
        return StatusLed::State::ERROR;

    if (_webHealth == SubsystemHealth::ERROR)
        return StatusLed::State::WARNING; // yellow blinking
    */
    if (_state.controlMode == ThrottleControlMode::AUTOMATIC)
        return StatusLed::State::RUNNING;

    if (!_motor.isReady())
        return StatusLed::State::BUSY;

    // 4 Normal idle
    return StatusLed::State::READY;
}

void ThrustStand::fillCalibrationJson(JsonDocument &doc)
{
    doc["version"] = THRUSTSTAND_APP_VERSION;

    doc["thrust"]["cal"] = getThrustCalFactor();

    auto torque = getTorqueCalibration();
    doc["torque"]["cal1"] = torque.cal1;
    doc["torque"]["cal2"] = torque.cal2;
    doc["torque"]["distance_mm"] = torque.distanceMM;

    doc["current"]["sensitivity"] = getCurrentSensitivity();
    doc["voltage"]["calibration"] = getVoltageCalibrationFactor();

    doc["motor"]["pwm_min_us"] = getMinPulseUs();
    doc["motor"]["pwm_max_us"] = getMaxPulseUs();
    //
    doc["motor"]["disarm_timeout_s"] = _autoDisarmMs / 1000UL;
    doc["motor"]["esc_driver_type"] = static_cast<uint8_t>(_escDriverType);
}

void ThrustStand::fillSystemStateJson(JsonDocument &doc) const
{
    const ThrustStandState s = getState();

    doc["thrust"] = static_cast<uint8_t>(s.thrustSensor);
    doc["torque"] = static_cast<uint8_t>(s.torqueSensor);
    doc["temperature"] = static_cast<uint8_t>(s.tempSensor);
    doc["rpm"] = static_cast<uint8_t>(s.rpmSensor);
    doc["current"] = static_cast<uint8_t>(s.currentSensor);
    doc["voltage"] = static_cast<uint8_t>(s.voltageSensor);
    doc["motor"] = static_cast<uint8_t>(s.motor);
    doc["motor_mount"] = static_cast<uint8_t>(s.motorMountState);
}

void ThrustStand::fillLiveSnapshot(JsonDocument &doc) const
{
    const test_data_t d = getCurrentDataSet();
    const SafetyState safetyState = getSafetyState();

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

void ThrustStand::fillSafetyJson(JsonDocument &doc) const
{

    // Limits
    doc["limits"]["throttle_percent"] = _safety.coreLimits.maxThrottlePercent;
    doc["limits"]["current_a"] = _safety.coreLimits.maxCurrentA;
    doc["limits"]["voltage_max_v"] = _safety.coreLimits.maxVoltageV;
    doc["limits"]["voltage_min_v"] = _safety.coreLimits.minVoltageV;
    doc["limits"]["thrust_gf"] = _safety.coreLimits.maxThrustGF;
    doc["limits"]["battery_cells"] = batteryPresetToCells(_safety.coreLimits.batteryPreset);
}

void ThrustStand::fillBatteryPresetJson(JsonDocument &doc) const
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

void ThrustStand::fillESCDriverJson(JsonDocument &doc) const
{
    JsonArray arr = doc.createNestedArray("esc_drivers");

    auto addDriver = [&](EscDriverType type, const char *label)
    {
        JsonObject obj = arr.createNestedObject();
        obj["id"] = static_cast<uint8_t>(type);
        obj["label"] = label;
    };

    // ---------- NONE ----------
    addDriver(EscDriverType::ESC_DRIVER_NONE,
              escDriverToString(EscDriverType::ESC_DRIVER_NONE));

    // ---------- PWM ----------
    addDriver(EscDriverType::ESC_DRIVER_PWM,
              escDriverToString(EscDriverType::ESC_DRIVER_PWM));

    // ---------- D-Shot variants ----------
    addDriver(EscDriverType::ESC_DRIVER_DSHOT150,
              escDriverToString(EscDriverType::ESC_DRIVER_DSHOT150));

    addDriver(EscDriverType::ESC_DRIVER_DSHOT300,
              escDriverToString(EscDriverType::ESC_DRIVER_DSHOT300));

    addDriver(EscDriverType::ESC_DRIVER_DSHOT600,
              escDriverToString(EscDriverType::ESC_DRIVER_DSHOT600));
}

void ThrustStand::updateEsc()
{
    _motor.updateEsc();
}
