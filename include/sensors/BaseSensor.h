/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SENSOR_BASE_H
#define SENSOR_BASE_H
#include "SensorState.h"

/**
 * @class BaseSensor
 * @brief Abstract base class for all sensors with unified state and data validity handling.
 *
 * Provides a minimal interface and common infrastructure for:
 * - Sensor lifecycle state tracking
 * - Data validity handling
 * - Update/error diagnostics
 *
 * Derived classes implement hardware-specific acquisition and processing.
 *
 * @section sensor_model Sensor Model
 *
 * A sensor operates in two parallel dimensions:
 *
 * 1. Lifecycle state (@ref SensorState)
 * 2. Data validity (hasValidData())
 *
 * These are intentionally separated:
 * - A sensor may be initialized but not yet provide valid data
 * - A sensor may remain initialized while temporarily producing invalid data
 *
 *
 * @section sensor_update Update Model
 *
 * - update() must be called periodically
 * - Each successful call increments the internal update counter
 * - Derived classes are responsible for:
 *   - Acquiring new data
 *   - Validating measurements
 *   - Updating data validity state
 *
 *
 * @section sensor_validity Data Validity
 *
 * - hasValidData() indicates whether the latest measurement is usable
 * - Invalid data does NOT necessarily imply a sensor error
 * - Persistent failures should increment the error counter
 *
 *
 * @section sensor_diagnostics Diagnostics
 *
 * - getUpdateCount(): total number of update() calls
 * - getErrorCount(): number of detected errors or invalid readings
 *
 *
 * @section sensor_responsibilities Responsibilities of Derived Classes
 *
 * Derived classes must:
 * - Implement update()
 * - Set state via setState()
 * - Maintain data validity via setDataValid()
 * - Increment error count on failures via incrementError()
 *
 * Derived classes must NOT:
 * - Expose partially invalid data without clearing validity flag
 *
 *
 * @note
 * This class does not define timing, filtering, or calibration behavior.
 */
class BaseSensor
{
public:
    virtual ~BaseSensor() = default;

    // Core sensor action
    /**
     * @brief Perform a sensor update cycle.
     *
     * @return true if update succeeded, false otherwise
     *
     * @note
     * - Should acquire new data and update internal state
     * - Should update data validity flag accordingly
     */
    virtual bool update() = 0;

    // Lifecycle

    /**
     * @brief Get current sensor state.
     */
    SensorState getState() const;

    /**
     * @brief Check if sensor is ready for operation.
     *
     * @return true if initialized and not in error state
     */
    bool isReady() const;

    /**
     * @brief Check if sensor has completed initialization.
     */
    bool isInitialized() const;

    // Data validity

    /**
     * @brief Check if latest sensor data is valid.
     *
     * @return true if current data can be used
     *
     * @note
     * - Independent of sensor state
     * - May be false even if sensor is initialized
     */
    bool hasValidData() const;

    // Diagnostics

    /**
     * @brief Get number of update cycles performed.
     */
    uint32_t getUpdateCount() const;

    /**
     * @brief Get number of detected errors.
     *
     * @note
     * Incremented via incrementError()
     */
    uint32_t getErrorCount() const;

protected:
    /**
     * @brief Construct base sensor.
     *
     * Initializes internal state and counters.
     */
    BaseSensor();

    // State control for derived classes

    /**
     * @brief Set sensor lifecycle state.
     *
     * @param state New sensor state
     */
    void setState(SensorState state);

    /**
     * @brief Set data validity flag.
     *
     * @param valid True if data is valid
     */
    void setDataValid(bool valid);

    /**
     * @brief Increment error counter.
     *
     * Should be called on failed updates or invalid readings.
     */
    void incrementError();

protected:
    SensorState _state;
    bool _dataValid;
    uint32_t _updateCount;
    uint32_t _errorCount;
};

#endif