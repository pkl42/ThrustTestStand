#ifndef THERMOCOUPLE_SENSOR_H
#define THERMOCOUPLE_SENSOR_H

#include "BaseSensor.h"
#include <Adafruit_MAX31855.h>

/**
 * @brief Thermocouple temperature sensor using MAX31855
 *
 * This class wraps the Adafruit_MAX31855 driver and integrates it into the
 * BaseSensor framework. It provides non-blocking temperature updates and
 * tracks data validity.
 */
class ThermocoupleSensor : public BaseSensor
{
public:
    ThermocoupleSensor() = default;
    ~ThermocoupleSensor() = default;

    /**
     * @brief Initialize the MAX31855 thermocouple interface
     *
     * This method configures the SPI-style pins used by the MAX31855.
     * It must be called once before calling update().
     *
     * @param clkPin Clock pin for MAX31855 (SCK)
     * @param csPin  Chip select pin (CS). If -1, CS is not used.
     * @param soPin  Serial data output pin (MISO). If -1, defaults are used.
     *
     * @return true if the sensor was successfully initialized,
     *         false if initialization failed
     */
    bool begin(int8_t clkPin, int8_t csPin = -1, int8_t soPin = -1);

    /**
     * @brief Read and update the latest thermocouple temperature
     *
     * This method should be called periodically from the main loop or a task.
     * It reads the MAX31855, updates the internal temperature value,
     * and updates the sensor validity state.
     *
     * @return true if a valid temperature sample was read and stored,
     *         false if a fault occurred or the data is invalid
     */
    bool update() override;

    /**
     * @brief Get the last measured temperature
     *
     * @return Temperature in degrees Celsius.
     *         The returned value is only valid if hasValidData() returns true.
     */
    float getTemperatureInCelsius() const;

private:
    const char *TAG = "ThermocoupleSensor";

    Adafruit_MAX31855 _tc{0, 0, 0}; ///< MAX31855 driver instance (pins set in begin)
    float _temperature{0.0f};       ///< Last valid temperature sample in °C
};

#endif // THERMOCOUPLE_SENSOR_H
