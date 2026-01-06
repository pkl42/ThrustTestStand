#ifndef THRUST_STAND_H
#define THRUST_STAND_H

#include "pinOut.h"
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
    uint32_t power;         ///< Number of power samples
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
 * @brief High-level interface for controlling and monitoring a thrust stand.
 *
 * Responsibilities:
 *  - Initialize and update all sensors
 *  - Control motor ESC and throttle
 *  - Acquire, log, and accumulate test data
 *  - Provide machine-level state snapshots
 *
 * Contains no UI or presentation logic.
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
     * @brief Tare all applicable sensors to zero.
     */
    void tareSensors();

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
     * @brief Get currently applied throttle.
     *
     * @return Current throttle percentage
     */
    float getCurrentThrottle() const { return motor.getCurrentThrottle(); }

    /**
     * @brief Set the throttle control mode.
     *
     * @param mode Desired throttle mode (manual or automatic)
     */
    void setControlMode(ThrottleControlMode mode);

    /**
     * @brief Get maximum allowed throttle percentage.
     */
    float getMaxThrottlePercent() const { return motor.getMaxThrottlePercent(); }

    /**
     * @brief Set maximum allowed throttle percentage.
     */
    void setMaxThrottlePercent(float maxPercent);

    /**
     * @brief Get current motor ESC state.
     */
    MotorESC::EscState getMotorState() const { return motor.getMountState(); }

    /**
     * @brief Configure the ESC pulse range.
     *
     * Validates min and max microsecond pulses.
     */
    bool setPulseRangeUs(uint16_t minPulseUs, uint16_t maxPulseUs);

    uint16_t getMinPulseUs() const { return motor.getMinPulseUs(); }
    uint16_t getMaxPulseUs() const { return motor.getMaxPulseUs(); }

    /**
     * @brief Check if the motor has reached the target RPM.
     */
    bool isMotorAtTargetSpeed() const { return motor.isAtTargetSpeed(); }

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
     * @brief Get current sensor sensitivity (volts per amp).
     */
    float getCurrentSensitivity() const { return currentSensor.getSensitivity(); }

    /**
     * @brief Set current sensor sensitivity (volts per amp).
     */
    void setCurrentSensitivity(float voltsPerAmp);

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

    ThrustSensor thrustSensor; ///< Thrust sensor instance
    TorqueSensor torqueSensor; ///< Torque sensor instance
    RpmSensor rpmSensor;       ///< RPM sensor instance
    MotorESC motor;            ///< Motor ESC controller

    BusVoltageADC voltageSensor;           ///< Voltage sensor instance
    CurrentACS758 currentSensor;           ///< Current sensor instance
    ThermocoupleSensor thermocoupleSensor; ///< Thermocouple sensor instance
    HardwareEstop hwEstop;                 ///< Emergency stop hardware

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
