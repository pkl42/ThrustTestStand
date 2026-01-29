/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TORQUE_SENSOR_H
#define TORQUE_SENSOR_H

#include "BaseSensor.h"
#include <HX711_ADC.h>

/**
 * @brief Phases of initialization for the TorqueSensor.
 *
 * This enum is used internally to manage the multi-step
 * initialization process for dual HX711 load cells.
 */
enum class TorqueInitPhase : uint8_t
{
    IDLE,      ///< No initialization in progress
    START_LC1, ///< Begin initialization of Load Cell 1
    WAIT_LC1,  ///< Wait for Load Cell 1 to settle
    START_LC2, ///< Begin initialization of Load Cell 2
    WAIT_LC2,  ///< Wait for Load Cell 2 to settle
    DONE,      ///< Initialization completed successfully
    ERROR      ///< Initialization encountered an error
};

/**
 * @brief Torque sensor using two HX711 load cells.
 *
 * This class handles dual load cells to measure torque, including:
 * - Initialization sequence
 * - Calibration
 * - Non-blocking periodic updates
 * - Torque calculation in N·cm
 *
 * @note Torque is calculated as the sum of absolute loads multiplied
 *       by lever arm distance. Ensure calibration and distanceMM are
 *       correctly set for accurate results.
 */
class TorqueSensor : public BaseSensor
{
public:
    /**
     * @brief Calibration parameters for the torque sensor.
     *
     * @note distanceMM is the effective lever arm in mm; small changes
     *       have a proportional effect on torque calculation.
     */
    struct Calibration
    {
        float cal1 = 1.f;       ///< Load Cell 1 calibration factor
        float cal2 = 1.f;       ///< Load Cell 2 calibration factor
        float distanceMM = 1.f; ///< Lever arm distance in mm
    };

    /**
     * @brief Construct a TorqueSensor with the given HX711 pins.
     *
     * Pins are fixed at construction time.
     *
     * @param doutPin1 Data pin for Load Cell 1
     * @param sckPin1  Clock pin for Load Cell 1
     * @param doutPin2 Data pin for Load Cell 2
     * @param sckPin2  Clock pin for Load Cell 2
     */
    TorqueSensor(uint8_t doutPin1, uint8_t sckPin1,
                 uint8_t doutPin2, uint8_t sckPin2);

    /**
     * @brief Initialize the load cells and optionally set calibration.
     *
     * Must be called before using update() or getTorqueNcm().
     *
     * @param cal1 Calibration factor for Load Cell 1
     * @param cal2 Calibration factor for Load Cell 2
     * @param distanceMM Lever arm distance in millimeters
     * @return true if initialization succeeded, false otherwise
     *
     * @warning Initialization is multi-step; calling update() before
     *          begin() completes may give invalid readings.
     */
    bool begin(float cal1 = 0.f, float cal2 = 0.f, float distanceMM = -1.f);

    /**
     * @brief Update the calibration factors.
     *
     * If a value is negative or zero, it will be ignored and the previous
     * calibration value remains.
     *
     * @param cal1 Load Cell 1 calibration factor
     * @param cal2 Load Cell 2 calibration factor
     * @param distanceMM Lever arm distance in mm
     *
     * @note Changing calibration values affects all subsequent torque readings.
     */
    void setCalibration(float cal1 = 0.f, float cal2 = 0.f, float distanceMM = -1.f);

    /**
     * @brief Retrieve the current calibration parameters.
     * @return Calibration struct with cal1, cal2, and distanceMM
     */
    Calibration getCalibration() const { return _calibration; }

    /**
     * @brief Perform a periodic update of the load cell readings.
     *
     * Should be called regularly (e.g., in a main loop or task).
     *
     * @return true if the sensor has valid data, false if not ready
     *
     * @note This method is non-blocking and may return false if the
     *       sensor is still initializing.
     * @warning Ensure update() is called frequently enough to capture
     *          load changes; otherwise torque readings may be stale.
     */
    bool update();

    /**
     * @brief Compute the current torque in N·cm.
     *
     * Torque is computed as the sum of absolute load cell values
     * multiplied by the lever arm distance and scaling factor.
     *
     * @return Torque in N·cm
     *
     * @note Make sure calibration factors and distanceMM are correct.
     *       Units of _lc1_load and _lc2_load are assumed consistent with calibration.
     */
    float getTorqueNcm() const
    {
        return ((abs(_lc1_load) + abs(_lc2_load)) * _calibration.distanceMM / 10. / 100.);
    }

    /**
     * @brief Get the current load measured by Load Cell 1.
     * @return Load value from LC1 (unit depends on calibration)
     */
    float getLoad1() const { return _lc1_load; }

    /**
     * @brief Get the current load measured by Load Cell 2.
     * @return Load value from LC2 (unit depends on calibration)
     */
    float getLoad2() const { return _lc2_load; }

    /**
     * @brief Tare both load cells, setting the current reading as zero.
     *
     * Useful to remove offset or preload from measurements.
     *
     * @note Should be called when no load is applied for best accuracy.
     *
     * @return true  If the zero offset was successfully determined and stored.
     * @return false If taring failed (e.g. sensor not ready, ADC unavailable,
     *               mutex not acquired). *
     */
    bool tare();

private:
    const char *TAG = "TorqueSensor"; ///< Logging tag
    HX711_ADC _lc1;                   ///< Load Cell 1 interface
    HX711_ADC _lc2;                   ///< Load Cell 2 interface

    TorqueInitPhase _initPhase = TorqueInitPhase::IDLE; ///< Current initialization phase

    float _lc1_load = 0.0f; ///< Latest load value from LC1
    float _lc2_load = 0.0f; ///< Latest load value from LC2

    Calibration _calibration; ///< Current calibration parameters
};

#endif // TORQUE_SENSOR_H
