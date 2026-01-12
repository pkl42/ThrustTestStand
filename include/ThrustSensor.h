#ifndef THRUST_SENSOR_H
#define THRUST_SENSOR_H

#include <HX711_ADC.h>
#include "BaseSensor.h"

/**
 * @brief Class to interface with a load cell via HX711 ADC to measure thrust.
 *
 * This class wraps HX711_ADC functionality and provides calibration,
 * tare, and periodic update methods. It implements the BaseSensor interface
 * for standardized sensor update handling.
 */
class ThrustSensor : public BaseSensor
{
public:
    /**
     * @brief Construct a new ThrustSensor object.
     *
     * @param doutPin Data output pin from HX711.
     * @param sckPin Serial clock pin for HX711.
     */
    ThrustSensor(uint8_t doutPin, uint8_t sckPin);

    /**
     * @brief Initialize the sensor and optionally set a calibration factor.
     *
     * Must be called before calling update() or reading thrust.
     *
     * @param calibration Optional calibration factor to convert raw units to physical units.
     * @return true if initialization was successful, false otherwise.
     */
    bool begin(float calibration = 0.f);

    /**
     * @brief Set the calibration factor for the load cell.
     *
     * The calibration factor scales raw readings to meaningful thrust values.
     *
     * @param calibration Calibration factor (unit-dependent, e.g., N per ADC unit).
     */
    void setCalibration(float calibration);

    /**
     * @brief Get the currently used calibration factor.
     *
     * @return float Current calibration factor.
     */
    float getCalibration() { return _loadCell.getCalFactor(); };

    /**
     * @brief Perform a non-blocking update of the thrust reading.
     *
     * Reads the HX711 ADC, applies calibration, and updates the internal thrust value.
     * Should be called periodically, e.g., in a main loop or sensor task.
     *
     * @return true if a new reading was successfully obtained, false otherwise.
     */
    bool update() override;

    /**
     * @brief Get the most recent thrust measurement.
     *
     * @return float Thrust value in calibrated units.
     */
    float getThrust() const
    {
        return _thrust;
    };

    /**
     * @brief Tare (zero) the sensor.
     *
     * Adjusts the sensor offset so that the current load reads as zero.
     * Useful to eliminate bias or pre-load effects.
     * 
     * @return true  If the zero offset was successfully determined and stored.
     * @return false If taring failed (e.g. sensor not ready, ADC unavailable,
     *               mutex not acquired). * 
     */
    bool tare();

private:
    const char *TAG = "ThrustSensor"; ///< Logging tag for ESP log messages
    HX711_ADC _loadCell;              ///< HX711 ADC interface object
    float _thrust = 1.0f;             ///< Last measured thrust value (calibrated)
};

#endif // THRUST_SENSOR_H
