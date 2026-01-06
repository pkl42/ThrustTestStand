#ifndef BUS_VOLTAGE_ADC_SENSOR_H
#define BUS_VOLTAGE_ADC_SENSOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#include "BaseSensor.h"

/**
 * @class BusVoltageADC
 * @brief Non-blocking, calibrated ADC-based bus voltage sensor (ESP32)
 *
 * This class implements a task-safe, non-blocking voltage measurement
 * using an ESP32 ADC input and a resistor divider.
 *
 * Features:
 *  - ESP32 ADC calibration compensation
 *  - Non-blocking update() sampling
 *  - Thread-safe access using FreeRTOS mutex
 *  - Designed for dynamic bus voltage measurement under load
 *
 * Typical use cases:
 *  - Battery voltage monitoring
 *  - Motor supply voltage measurement
 *  - Power calculation together with a current sensor
 */
class BusVoltageADC : public BaseSensor
{
public:
    /**
     * @brief Construct a new BusVoltageADC sensor instance
     */
    BusVoltageADC();

    /**
     * @brief Destroy the BusVoltageADC sensor instance
     */
    ~BusVoltageADC();

    /**
     * @brief Initialize the bus voltage ADC sensor
     *
     * Configures the ESP32 ADC, voltage divider scaling,
     * and calibration characteristics.
     *
     * @param adcPin   Arduino ADC pin connected to the voltage divider
     * @return true if initialization succeeded
     */
    bool begin(uint8_t adcPin);

    /**
     * @brief set top and bottom resistor values
     *
     * @param rTop     Top resistor value of the voltage divider (Ohms)
     * @param rBottom  Bottom resistor value of the voltage divider (Ohms)
     */

    void setResistorvalues(float rTop, float rBottom);

    float getTopResistor() const { return _rTop;};
    float getBottomResistor() const { return _rBottom;};
    

    /**
     * @brief Perform a non-blocking ADC sample and update the bus voltage
     *
     * This method should be called periodically from a task or main loop.
     * It performs ADC sampling, calibration compensation,
     * and voltage divider scaling.
     *
     * @return true  If a new sample was successfully acquired and the internal
     *               voltage value was updated with valid data.
     * @return false If the sample could not be completed or was invalid
     *               (e.g. ADC not ready, conversion error, out-of-range reading).
     */
    bool update() override;

    /**
     * @brief Get the latest measured bus voltage
     *
     * Thread-safe and non-blocking.
     *
     * @return Bus voltage in volts
     */
    float getVoltage_V();

private:
    /**
     * @brief Convert raw ADC reading to scaled bus voltage
     *
     * Applies ESP32 ADC calibration (if available) and
     * resistor divider scaling.
     *
     * @param raw Raw ADC reading
     * @return Scaled bus voltage in volts
     */
    float adcToVoltage(uint32_t raw);

    /**
     * @brief Voltage divider scaling factor
     *
     * scale = (Rtop + Rbottom) / Rbottom
     */
    float _scale = 1.0f;

    /**
     * @brief ADC pin connected to the voltage divider output
     */
    uint8_t _pin = 0;

    /**
     * @brief ESP32 ADC1 channel corresponding to the pin
     */
    adc1_channel_t _adcChannel;

    /**
     * @brief Top resistor of the voltage divider (Ohms)
     */
    float _rTop = 10e3f;

    /**
     * @brief Bottom resistor of the voltage divider (Ohms)
     */
    float _rBottom = 1e3f;

    /**
     * @brief Latest measured bus voltage
     */
    float _voltage = 0.0f;

    /**
     * @brief ESP32 ADC calibration characteristics
     */
    esp_adc_cal_characteristics_t _adcChars;

    /**
     * @brief Indicates whether ADC calibration data is available
     */
    bool _adcCalibrated = false;

    /**
     * @brief FreeRTOS mutex for thread-safe access
     */
    SemaphoreHandle_t _mutex = nullptr;

    /**
     * @brief Logging tag
     */
    static constexpr const char *TAG = "BusVoltageADC";
};

#endif // BUS_VOLTAGE_ADC_SENSOR_H
