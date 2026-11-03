/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BASE_TEMPERATURE_SENSOR_H
#define BASE_TEMPERATURE_SENSOR_H

#include "BaseSensor.h"

/**
 * @class BaseTemperatureSensor
 * @brief Abstract base class for temperature sensors.
 *
 * Extends @ref BaseSensor with temperature-specific functionality,
 * including calibrated temperature access and unit conversion.
 *
 * @section temp_model Temperature Model
 *
 * - Temperature is internally stored in degrees Celsius
 * - Calibration is applied as a linear transformation:
 *
 *   T_calibrated = T_raw * coefficient + offset
 *
 * - The stored value represents the latest processed (calibrated) sample
 *
 *
 * @section temp_validity Data Validity
 *
 * - getTemperatureInCelsius() is only valid if hasValidData() returns true
 * - Invalid readings must not be used for control or safety decisions
 *
 *
 * @section temp_calibration Calibration
 *
 * Calibration compensates:
 * - Sensor gain error (coefficient)
 * - Sensor offset error (offset)
 *
 * Default calibration:
 * - coefficient = 1.0 (no scaling)
 * - offset = 0.0 (no offset)
 *
 *
 * @note
 * This class does not define how temperature is measured.
 * Derived classes implement hardware-specific acquisition.
 *
 * @see BaseSensor
 */
class BaseTemperatureSensor : public BaseSensor
{
public:
    /**
     * @brief Linear calibration parameters.
     *
     * Applies a scaling and offset correction to raw temperature values.
     */
    struct Calibration
    {
        float coefficient = 1.f; ///< Temperature-Coefficient
        float offset = 0.f;      ///< Sensor-Offset
    };

    virtual ~BaseTemperatureSensor() = default;

    /**
     * @brief Set calibration parameters.
     *
     * @param coefficient Multiplicative correction factor
     * @param offset      Additive offset in °C
     */
    virtual void setCalibration(float coefficient = 1.f, float offset = 0.f);

    /**
     * @brief Get current calibration parameters.
     */
    virtual Calibration getCalibration() const;

    /**
     * @brief Get the last measured temperature in Celsius.
     *
     * @return Temperature in °C
     *
     * @note
     * - Only valid if hasValidData() returns true
     * - Represents the last processed (calibrated) value
     */
    virtual float getTemperatureInCelsius() const;

    /**
     * @brief Get temperature in Kelvin.
     *
     * @return Temperature in K
     *
     * @note
     * - Derived from Celsius value
     * - Same validity constraints apply
     */
    virtual float getTemperatureKelvin() const
    {
        return getTemperatureInCelsius() + 273.15f;
    }

protected:
    float _temperature{0.0f}; ///< Last valid calibrated temperature [°C]

    Calibration _calibration; ///< Active calibration parameters
};

#endif // BASE_TEMPERATURE_SENSOR_H