/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef THRUST_STAND_H
#define THRUST_STAND_H

#include <ArduinoJson.h>
#include "core/Config.h"
#include "core/SafetyLimits.h"
#include "actuators/MotorESC.h"
#include "actuators/esc/DShotEscDriver.h"
#include "actuators/esc/PwmEscDriver.h"

#include "sensors/RPMSensor.h"
#include "sensors/ThrustSensor.h"
#include "sensors/TorqueSensor.h"

// #include "current_ina226_sensor.h"
#include "sensors/CurrentACS758Sensor.h"
#include "sensors/BusVoltageADCSensor.h"
#include "sensors/TemperatureSensor.h"
#include "safety/HardwareEstop.h"
#include "actuators/StatusLed.h"

/**
 * @brief Data container for a single set of thrust stand measurements.
 *
 * Contains all machine-level numeric data from sensors and calculations.
 * Used for:
 *  - Current live data snapshot
 */
typedef struct
{
    float throttle_cmd;  ///< Commanded throttle percentage (0.0 – 100.0)
    float throttle_act;  ///< Actual throttle feedback from ESC [%]
    float thrust;        ///< Thrust measurement [N]
    float torque;        ///< Calculated torque [Nm]
    float torque_cell_1; ///< Raw torque load cell 1 value
    float torque_cell_2; ///< Raw torque load cell 2 value
    float voltage;       ///< Bus voltage [V]
    float current;       ///< Measured current [A]
    float power;         ///< Measured or calculated Power [W]
    float thrust_ratio;  ///< calculated thrust ratio [gf/W]
    float rpm;           ///< Motor speed in RPM
    float temperature;   ///< Measured temperature [°C]
} test_data_t;

struct sensor_stats_t
{
    float mean;
    float M2;
    float min;
    float max;
    uint32_t n = 0;
};

/**
 * @brief Accumulated statistics data.
 *
 * Stores the summed values and the corresponding sample counts for each sensor channel.
 */
struct test_data_accu_t
{
    uint32_t t_ms;      // timestamp ms
    uint16_t stepIndex; // protocol step, informational only
    sensor_stats_t throttle;
    sensor_stats_t thrust;
    sensor_stats_t torque;
    sensor_stats_t torque_cell_1;
    sensor_stats_t torque_cell_2;
    sensor_stats_t voltage;
    sensor_stats_t current;
    sensor_stats_t power;
    sensor_stats_t thrust_ratio;
    sensor_stats_t rpm;
    sensor_stats_t temperature;
};

/**
 * @brief Source of throttle commands.
 */
enum class ThrottleSource
{
    MANUAL, ///< Operator or UI control
    TEST    ///< Automated test sequence
};

/**
 * @brief Throttle control mode.
 */
enum class ThrottleControlMode
{
    MANUAL,   ///< Direct throttle control
    AUTOMATIC ///< Throttle controlled by test sequence
};

/**
 * @brief Aggregated machine-level state of the thrust stand.
 *
 * Contains the current state of all sensors and actuators. Intended for
 * monitoring interfaces or system APIs. Contains no presentation logic.
 */
struct ThrustStandState
{
    SensorState thrustSensor;  ///< Thrust sensor state
    SensorState torqueSensor;  ///< Torque sensor state
    SensorState chamberTemp;   ///< Chamber temperature sensor state
    SensorState rpmSensor;     ///< RPM sensor state
    SensorState currentSensor; ///< current sensor state
    SensorState voltageSensor; ///< voltage sensor state
    SensorState tempSensor;    ///< temperature sensor state

    ActuatorState motor;                ///< Motor actuator state
    MotorESC::EscState motorMountState; ///< Motor ESC / mount state

    ThrottleControlMode controlMode = ThrottleControlMode::MANUAL;
    uint32_t idleTimeMs = 0;
};

/**
 * @class ThrustStand
 * @brief High-level façade for controlling and monitoring a thrust stand.
 *
 * The ThrustStand class acts as the central coordination layer of the system.
 * It aggregates sensors, actuators, and calibration data into a single,
 * machine-level interface.
 *
 * Key responsibilities:
 *  - Initialize and update all sensors and actuators
 *  - Control the motor ESC and throttle behavior
 *  - Acquire, accumulate, and provide measurement data
 *  - Manage calibration parameters and persist them in non-volatile storage
 *  - Expose a stable API for UI, logging, and test automation
 *
 * Design notes:
 *  - Sensor classes are intentionally kept lightweight and stateless
 *    with respect to persistence.
 *  - This class acts as a façade, centralizing configuration, calibration
 *    workflows, and NVS storage.
 *  - Contains no UI, networking, or presentation logic.
 */

class ThrustStand
{
public:
    /**
     * @brief Construct a new ThrustStand instance.
     */
    ThrustStand();

    ~ThrustStand();

    /**
     * @brief Initialize thrust stand hardware and subsystems.
     *
     * Must be called before using any other functions.
     *
     * @return true if initialization succeeded
     */
    bool init();

    /**
     * @brief Initialize all connected sensors.
     *
     * @param force forces initialization, where sensor allows this
     *
     * @return true if all sensors initialized successfully
     */
    bool init_sensors(bool force = false);

    /**
     * @brief Arm the motor ESC.
     *
     * Enables the ESC output and transitions the motor into an armed state,
     * provided all safety conditions are satisfied.
     *
     * @return true if the motor was successfully armed
     * @return false if arming was rejected due to safety conditions or ESC state
     *
     * @see isSafeToArm()
     */
    bool armMotor();

    /**
     * @brief Disarm the motor ESC.
     *
     * Immediately disables ESC output and forces the motor into a safe,
     * non-armed state regardless of throttle command.
     *
     * @return true if the motor was successfully disarmed
     */
    bool disarmMotor();

    /**
     * @brief Tare all installed sensors of the thrust stand.
     *
     * Calls the tare() function of all available sensors (thrust, torque,
     * and optional current sensor). The operation is considered successful
     * only if all individual taring operations succeed.
     *
     * @return true  If all sensors were successfully tared.
     * @return false If one or more sensors failed to tare.
     */
    bool tareSensors();

    /**
     * @brief Update all sensors and internal state.
     *
     * Should be called periodically in the main loop.
     *
     * @return true if update succeeded
     */
    bool update();

    void updateEsc();

    /**
     * @brief Set the motor throttle.
     *
     * @param throttle Throttle percentage (0.0 – 100.0)
     * @param source Source of the throttle command (manual or test)
     * @return Applied throttle value
     */
    float setThrottle(float throttle,
                      ThrottleSource source = ThrottleSource::MANUAL);

    /**
     * @brief Set the motor throttle with optional ramping.
     *
     * @param throttle Target throttle percentage
     * @param smooth Enable smooth ramping to target
     * @param accelTimeMs Time to reach target throttle in milliseconds
     * @param source Source of the throttle command
     * @return Applied throttle value
     */
    float setThrottle(float throttle,
                      bool smooth,
                      unsigned long accelTimeMs,
                      ThrottleSource source = ThrottleSource::MANUAL);

    /**
     * @brief Get current sensor sensitivity.
     *
     * Returns the effective sensitivity of the current sensor in volts per ampere
     * at the current sensor supply voltage.
     *
     * @return Sensor sensitivity in V/A
     */
    float getCurrentSensitivity() const { return _currentSensor.getSensitivity(); }

    /**
     * @brief Set the throttle control mode.
     *
     * @param mode Desired throttle mode (manual or automatic)
     */
    void setControlMode(ThrottleControlMode mode);

    /**
     * @brief Configure the maximum allowed throttle limit.
     *
     * Defines an upper bound for the commanded throttle value. The controller
     * shall clamp any requested throttle to this limit.
     *
     * @note This limit is applied continuously during normal operation.
     * @note A value of 100.0f disables throttle limiting.
     */
    void setThrottleLimitPercent(float percent, SafetyTripSource source);
    /**
     * @brief Set the active battery configuration preset.
     *
     * Stores the selected battery preset for informational and contextual use
     * within the system (e.g. UI display, logging, validation hints).
     *
     * @note This function does NOT modify any safety limits such as minimum or
     *       maximum battery voltage.
     * @note Selecting a battery preset alone has no direct effect on runtime
     *       behavior or safety enforcement.
     * @note Application of preset-derived default limits, if desired, must be
     *       performed explicitly via a separate mechanism.
     *
     * @param preset Selected battery preset (e.g. 3S, 4S, 5S).
     * @param source CORE_LIMIT or TEST_LIMIT
     */

    void setBatteryPreset(BatteryPreset preset,
                          SafetyTripSource source);

    /**
     * @brief Configure the maximum allowed motor current limit.
     *
     * Defines the absolute upper limit for measured motor current.
     *
     * @note Currently, this value is stored as a configuration parameter only.
     * @todo Implement runtime monitoring and trigger
     *       SafetyTripReason::SAFETY_TRIP_OVER_CURRENT when exceeded.
     */
    void setCurrentLimitA(float maxCurrentA, SafetyTripSource source);

    /**
     * @brief Configure the maximum allowed motor supply voltage limit.
     *
     * Defines the absolute upper limit for measured motor supply voltage.
     *
     * @note Currently, this value is stored as a configuration parameter only.
     * @todo Implement runtime monitoring and trigger
     *       SafetyTripReason::SAFETY_TRIP_OVER_VOLTAGE when exceeded.
     */
    void setVoltageLimitMaxV(float maxVoltageV, SafetyTripSource source);

    /**
     * @brief Configure the minimum allowed motor supply voltage limit.
     *
     * Defines the lower bound for measured motor supply voltage.
     *
     * @note Currently stored as a configuration parameter only.
     * @todo Implement runtime monitoring and trigger
     *       SafetyTripReason::SAFETY_TRIP_UNDER_VOLTAGE when violated.
     */
    void setVoltageLimitMinV(float minVoltageV, SafetyTripSource source);

    /**
     * @brief Configure the maximum allowed system temperature limit.
     *
     * Defines the upper limit for measured temperature from the thermocouple
     * sensor.
     *
     * @note Currently stored as a configuration parameter only.
     * @todo Implement runtime monitoring and trigger
     *       SafetyTripReason::SAFETY_TRIP_OVER_TEMPERATURE when exceeded.
     */
    void setTemperatureLimitC(float maxTemperatureC, SafetyTripSource source);

    /**
     * @brief Configure the maximum allowed thrust limit.
     *
     * Defines the absolute upper limit for measured thrust force in gf.
     * When the measured thrust exceeds this limit, the system should
     * trigger a safety trip (SAFETY_TRIP_OVER_THRUST).
     *
     * @param maxThrustGF Maximum allowed thrust in gram-force.
     *
     * @note Currently, this value is stored as a configuration parameter only.
     * @todo Implement runtime monitoring to trigger
     *       SafetyTripReason::SAFETY_TRIP_OVER_THRUST when exceeded.
     */
    void setThrustLimitGF(float maxThrustGF, SafetyTripSource source);

    /**
     * @brief Manually clear an active safety trip.
     *
     * Resets the _safety.state.tripped flag and the associated reason/value.
     * After calling this, the motor and throttle control may be re-enabled.
     *
     * @note Only clears the trip on the software side. Motors should still
     *       remain disabled until user acknowledges and/or restarts a test.
     */
    void clearSafetyTrip();

    /**
     * @brief Check all safety limits against the current sensor readings.
     *
     * Compares measured values (throttle, current, voltage, thrust, temperature, etc.)
     * with the configured limits and sets _safety.state.tripped if any limit is exceeded.
     *
     * @return true if all limits are within safe ranges, false if a safety trip occurred.
     *
     * @note This function is intended to be called in the main update loop.
     * @todo Extend to include additional safety checks (e.g., actuator health, sensor plausibility).
     */
    bool checkSafetyLimits();

    bool isSafetyTriggered() const { return _safety.state.tripped; };

    SafetyState getSafetyState() const { return _safety.state; }

    /**
     * @brief Check whether the propeller cage is open.
     *
     * This method reads the digital input connected to the normally-closed
     * propeller cage switch. A LOW reading indicates that the cage is open
     * or disconnected, which is considered an unsafe condition.
     *
     * @return true if the cage is open (unsafe), false if closed (safe).
     *
     * @note The cage switch is a fail-safe NC switch. If disconnected, this
     *       function will return true (unsafe).
     */
    bool isCageOpen() const;

    /**
     * @brief Determine if it is safe to arm the motor.
     *
     * Combines hardware interlocks to ensure the system can be safely armed.
     * The motor may only be armed if the E-Stop is not triggered and the
     * propeller cage is closed.
     *
     * @return true if it is safe to arm the motor, false otherwise.
     *
     * @see isCageOpen()
     * @see HardwareEstop::isTriggered()
     */
    bool isSafeToArm() const;

    /**
     * @brief Get current motor ESC state.
     */
    MotorESC::EscState getMotorState() const { return _motor.getMountState(); }

    /**
     * @brief Configure the ESC pulse range.
     *
     * Validates min and max microsecond pulses.
     */
    bool setPulseRangeUs(uint16_t minPulseUs, uint16_t maxPulseUs);

    uint16_t getMinPulseUs() const { return _motor.getMinPulseUs(); }
    uint16_t getMaxPulseUs() const { return _motor.getMaxPulseUs(); }

    bool switchDriver(EscDriverType type);

    /**
     * @brief Check if the motor has reached the target RPM.
     */
    bool isMotorAtTargetSpeed() const { return _motor.isAtTargetSpeed(); }

    /**
     * @brief Configure automatic motor disarm timeout.
     *
     * Specifies the duration of inactivity after which the motor will be
     * automatically disarmed if no throttle activity is detected.
     *
     * @param autoDisarmS Timeout in seconds
     */
    void setAutoDisarmTimeOut(uint32_t autoDisarmS);

    /**
     * @brief Get thrust sensor calibration factor.
     */
    float getThrustCalFactor();

    /**
     * @brief Set thrust sensor calibration factor.
     */
    void setThrustCalFactor(float factor);

    /**
     * @brief Set torque sensor calibration parameters.
     *
     * @param cal1 Calibration factor for cell 1
     * @param cal2 Calibration factor for cell 2
     * @param distance Lever arm distance [m]
     */
    void setTorqueCalibration(float cal1 = 0, float cal2 = 0, float distance = 0);

    /**
     * @brief Get current torque sensor calibration.
     */
    TorqueSensor::Calibration getTorqueCalibration();

    /**
     * @brief Set current sensor sensitivity.
     *
     * Updates the sensitivity used by the current sensor for converting measured
     * voltage into amperes.
     *
     * The value is:
     *  - Forwarded to the underlying CurrentACS758 sensor
     *  - Persistently stored in non-volatile storage (NVS)
     *
     * This method provides a centralized configuration interface and should be
     * preferred over directly accessing the sensor.
     *
     * @param voltsPerAmp Sensor sensitivity in volts per ampere
     */
    void setCurrentSensitivity(float voltsPerAmp);
    /**
     * @brief Automatically calibrate the current sensor using a known actual current.
     *
     * Performs a sensitivity calibration of the current sensor by comparing the
     * measured current against a known actual current value (e.g. from a laboratory
     * power supply).
     *
     * The calibration uses the ratio:
     * @code
     * newSensitivity = oldSensitivity * (measuredCurrent / actualCurrent)
     * @endcode
     *
     * If @p measuredCurrent_A is not provided (NaN), the current measurement is
     * automatically acquired from the sensor. This enables frontend-driven
     * calibration workflows without exposing ADC voltages or sensor internals.
     *
     * The resulting sensitivity is:
     *  - Applied to the underlying CurrentACS758 sensor
     *  - Persistently stored in non-volatile storage (NVS)
     *
     * @note The current sensor must be properly tared before calibration.
     * @note Calibration should be performed at a sufficiently high current
     *       to reduce noise influence.
     *
     * @param actualCurrent_A   Known actual current in amperes (must be > 0)
     * @param measuredCurrent_A Optional measured current in amperes; if NaN,
     *                          the value is automatically obtained
     */
    void autoCalibrateCurrentSensor(float actualCurrent_A,
                                    float measuredCurrent_A = NAN);

    /**
     * @brief Set the bus voltage calibration factor
     *
     * This method updates the empirical gain calibration used for
     * bus voltage measurement. The calibration factor compensates
     * ADC gain error and resistor tolerance.
     *
     * The value is:
     *  - Persistently stored in NVS
     *  - Forwarded to the underlying BusVoltageADC sensor
     *
     * This method acts as a façade, providing centralized configuration
     * management while keeping sensor implementations lightweight.
     *
     * Typical values are close to 1.0.
     *
     * @param calibrationFactor Gain correction factor
     */
    bool setVoltageCalibrationFactor(float calibrationFactor);

    float getVoltageCalibrationFactor() const { return (_voltageSensor.getCalibrationFactor()); };

    /**
     * @brief Get current live sensor data snapshot.
     *
     * @return Current measurement values
     */
    test_data_t getCurrentDataSet() const { return _currentDataSet; }

    /**
     * @brief Get cumulative data snapshot.
     *
     * @return Accumulated measurement values and sample counts
     */
    test_data_accu_t getAccuDataSet() const { return _accuDataSet; }

    /**
     * @brief Start accumulating statistics for averaging.
     */
    void startAccumulation();

    /**
     * @brief Stop accumulating statistics.
     */
    void stopAccumulation();

    /**
     * @brief Reset cumulative data and sample counters.
     */
    void resetAccumulativeData();

    /**
     * @brief Get the machine-level state snapshot.
     *
     * Non-blocking, returns the last updated state without new hardware access.
     *
     * @return Current thrust stand state
     */
    ThrustStandState getState() const { return _state; };

    /**
     * @brief Reset all stored configuration to defaults.
     */
    void resetNVS();

    /**
     * @brief Fill JSON document with calibration parameters.
     *
     * Populates the provided JSON document with all persisted calibration
     * values, including thrust, torque, current, and voltage calibration
     * factors.
     *
     * Intended for UI display, backup, or diagnostic export.
     *
     * @param doc JSON document to populate
     */
    void fillCalibrationJson(JsonDocument &doc);

    /**
     * @brief Fill JSON document with a live measurement snapshot.
     *
     * Contains instantaneous values and safety state.
     */
    void fillLiveSnapshot(JsonDocument &doc) const;

    /**
     * @brief Fill JSON document with system state information.
     *
     * Includes sensor presence, motor state, and motor mount status.
     */
    void fillSystemStateJson(JsonDocument &doc) const;

    /**
     * @brief Fill JSON document with safety limits and current safety state.
     *
     * Includes throttle/current/voltage/thrust limits and trip state.
     */
    void fillSafetyJson(JsonDocument &doc) const;

    /**
     * @brief Fill JSON document with battery preset list.
     *
     *
     */
    void fillBatteryPresetJson(JsonDocument &doc) const;

    /**
     * @brief Fill JSON document with esc protocol list
     *
     *
     */
    void fillESCDriverJson(JsonDocument &doc) const;

private:
    static const char *TAG; ///< Logging tag

    ThrustStandState _state;

    static test_data_t _currentDataSet;   ///< Current live data snapshot
    static test_data_accu_t _accuDataSet; ///< Cumulative data for statistics

    bool _accumulate_stats = false;  ///< Enable statistics accumulation
    uint32_t _autoDisarmMs = 120000; // 2 minutes

    ThrustSensor _thrustSensor; ///< Thrust sensor instance
    TorqueSensor _torqueSensor; ///< Torque sensor instance
    RpmSensor _rpmSensor;       ///< RPM sensor instance

    // Driver objects (stay alive as long as _motor exists)
    // ESC drivers first, then motor, important for class construction sequence
    PwmEscDriver *_pwmDriver = nullptr;
    DShotEscDriver *_dshotDriver = nullptr;
    EscDriverType _escDriverType;

    MotorESC _motor; ///< Motor ESC controller

    BusVoltageADC _voltageSensor;  ///< Voltage sensor instance
    CurrentACS758 _currentSensor;  ///< Current sensor instance
    TemperatureSensor _tempSensor; ///< Temperature sensor instance
    HardwareEstop _hwEstop;        ///< Emergency stop hardware

    /**
     * @brief Derive the appropriate status LED state.
     *
     * Computes the LED state based on system status, safety conditions,
     * and motor state.
     *
     * @return Derived LED state
     */
    StatusLed::State deriveLedState() const;

    /**
     * @brief Update the status LED output.
     *
     * Applies the derived LED state to the physical status LED.
     */
    void updateStatusLed();

    StatusLed _statusLed;

    /**
     * @brief Map a safety trip reason to a high-level actuator state.
     *
     * @param reason Safety trip reason
     * @return Corresponding actuator state
     */
    ActuatorState mapTripToState(SafetyTripReason reason);

    /**
     * @brief Internal update of all sensors.
     *
     * @return true if update succeeded
     */
    bool updateSensors();
    /**
     * @brief Update thrust sensor measurement.
     */
    bool updateThrust();

    /**
     * @brief Update torque sensor measurement.
     */
    bool updateTorque();

    /**
     * @brief Update RPM sensor measurement.
     */
    bool updateRPM();

    /**
     * @brief Update motor current measurement.
     */
    bool updateCurrent();

    /**
     * @brief Update bus voltage measurement.
     */
    bool updateVoltage();

    /**
     * @brief Update temperature measurement.
     */
    bool updateTemperature();

    /**
     * @brief Update derived sensor quantities.
     *
     * Computes values derived from raw sensor inputs, such as electrical
     * power or efficiency metrics.
     */
    bool updateDerivedSensorData();

    /**
     * @brief Update throttle control logic.
     *
     * Applies throttle ramping, clamping, and control-mode logic before
     * forwarding commands to the ESC.
     */
    bool updateThottle();
    /**
     * @brief Update idle activity tracking.
     *
     * Tracks system activity to determine whether the system is idle.
     */
    void updateIdleActivity();

    /**
     * @brief Check and trigger automatic motor disarm on inactivity.
     */
    void checkAutoDisarmOnIdle();

    ThrustStandSafety _safety;
    /**
     * @brief Trigger a safety trip condition.
     *
     * Records the trip source, reason, and measured value, and transitions
     * the system into a safe, tripped state.
     *
     * @param source Origin of the safety trip
     * @param reason Trip reason
     * @param value  Measured value that caused the trip
     */
    void triggerSafetyTrip(SafetyTripSource source,
                           SafetyTripReason reason,
                           float value);

    /**
     * @brief Load configuration from non-volatile storage.
     */
    void loadConfig();

    /**
     * @brief Save configuration to non-volatile storage.
     */
    void saveConfig();
};

#endif // THRUST_STAND_H