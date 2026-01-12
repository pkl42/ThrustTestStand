#ifndef THRUST_STAND_H
#define THRUST_STAND_H

#include "Config.h"
#include "MotorESC.h"
#include "RPMSensor.h"
#include "ThrustSensor.h"
#include "TorqueSensor.h"

// #include "current_ina226_sensor.h"
#include "CurrentACS758Sensor.h"
#include "BusVoltageADCSensor.h"
#include "ThermocoupleSensor.h"
#include "HardwareEstop.h"
#include "StatusLed.h"

/**
 * @brief Data container for a single set of thrust stand measurements.
 *
 * Contains all machine-level numeric data from sensors and calculations.
 * Used for:
 *  - Current live data snapshot
 *  - Accumulated/cumulative statistics
 *  - Averaged values via getAverage()
 */
typedef struct
{
    float throttle;        ///< Throttle percentage (0.0 – 100.0)
    float thrust;          ///< Thrust measurement [N]
    float torque;          ///< Calculated torque [Nm]
    float torque_cell_1;   ///< Raw torque load cell 1 value
    float torque_cell_2;   ///< Raw torque load cell 2 value
    float voltage;         ///< Bus voltage [V]
    float current;         ///< Measured current [A]
    float power;           ///< Calculated electrical power [W]
    float thrust_ratio;    ///< Calculated thrust /electrical power [W]
    float rpm;             ///< Motor speed in RPM
    float temperature;     ///< Measured temperature [°C]
    float temperature_max; ///< Maximum recorded temperature [°C]
} test_data_t;

/**
 * @brief Per-sensor sample counters for statistics accumulation.
 *
 * Tracks the number of samples taken for each sensor channel.
 */
typedef struct
{
    uint32_t thrust;        ///< Number of thrust samples
    uint32_t torque;        ///< Number of torque samples
    uint32_t torque_cell_1; ///< Number of torque cell 1 samples
    uint32_t torque_cell_2; ///< Number of torque cell 2 samples
    uint32_t voltage;       ///< Number of voltage samples
    uint32_t current;       ///< Number of current samples
    uint32_t rpm;           ///< Number of RPM samples
    uint32_t temperature;   ///< Number of temperature samples
} test_data_samples_t;

/**
 * @brief Accumulated statistics data.
 *
 * Stores the summed values and the corresponding sample counts for each sensor channel.
 */
typedef struct
{
    test_data_t values;          ///< Accumulated sums
    test_data_samples_t samples; ///< Number of samples per sensor
} test_data_accu_t;

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
    SensorState thrustSensor;       ///< Thrust sensor state
    SensorState torqueSensor;       ///< Torque sensor state
    SensorState chamberTemp;        ///< Chamber temperature sensor state
    SensorState rpmSensor;          ///< RPM sensor state
    SensorState currentSensor;      ///< current sensor state
    SensorState voltageSensor;      ///< voltage sensor state
    SensorState thermocoupleSensor; ///< Thermocouple sensor state

    ActuatorState motor;                ///< Motor actuator state
    MotorESC::EscState motorMountState; ///< Motor ESC / mount state

    ThrottleControlMode controlMode = ThrottleControlMode::MANUAL;
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
     * @return true if all sensors initialized successfully
     */
    bool init_sensors();

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
     * @brief Get maximum allowed throttle percentage.
     */
    float getMaxThrottlePercent() const { return _motor.getMaxThrottlePercent(); }

    /**
     * @brief Set maximum allowed throttle percentage.
     */
    void setMaxThrottlePercent(float maxPercent);

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

    /**
     * @brief Check if the motor has reached the target RPM.
     */
    bool isMotorAtTargetSpeed() const { return _motor.isAtTargetSpeed(); }

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
     * @brief Log the current live data set to console or storage.
     */
    void log_current_data_set();

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
    test_data_accu_t getCumDataSet() const { return _cumDataSet; }

    /**
     * @brief Get averaged values based on accumulated data.
     */
    test_data_t getAverage() const;

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
    void resetCumulativeData();

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

private:
    static const char *TAG; ///< Logging tag

    ThrustStandState _state;

    static test_data_t _currentDataSet;  ///< Current live data snapshot
    static test_data_accu_t _cumDataSet; ///< Cumulative data for statistics

    bool _accumulate_stats = false; ///< Enable statistics accumulation

    ThrustSensor _thrustSensor; ///< Thrust sensor instance
    TorqueSensor _torqueSensor; ///< Torque sensor instance
    RpmSensor _rpmSensor;       ///< RPM sensor instance
    MotorESC _motor;            ///< Motor ESC controller

    BusVoltageADC _voltageSensor;           ///< Voltage sensor instance
    CurrentACS758 _currentSensor;           ///< Current sensor instance
    ThermocoupleSensor _thermocoupleSensor; ///< Thermocouple sensor instance
    HardwareEstop _hwEstop;                 ///< Emergency stop hardware

    StatusLed::State deriveLedState() const;
    void updateStatusLed();
    StatusLed _statusLed;

    /**
     * @brief Internal update of all sensors.
     *
     * @return true if update succeeded
     */
    bool updateSensors();
    bool updateThrust();
    bool updateTorque();
    bool updateRPM();
    bool updateCurrent();
    bool updateVoltage();
    bool updateTemperature();

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
