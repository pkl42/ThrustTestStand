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
 * This class implements a task-safe, non-blocking bus voltage measurement
 * using an ESP32 ADC input and an external resistor voltage divider.
 *
 * Measurement pipeline:
 *   raw ADC → ESP32 ADC calibration → voltage divider scaling → gain calibration
 *
 * The resulting value represents the physical bus voltage in volts.
 *
 * Features:
 *  - ESP32 ADC calibration compensation (eFuse / Vref)
 *  - Hardware voltage divider scaling
 *  - Empirical gain calibration to compensate ADC and resistor tolerances
 *  - Non-blocking update() sampling
 *  - Thread-safe access using FreeRTOS mutex
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
     *
     * Initializes internal state. Hardware configuration is performed
     * by calling begin().
     */
    BusVoltageADC();

    /**
     * @brief Destroy the BusVoltageADC sensor instance
     */
    ~BusVoltageADC();

    /**
     * @brief Initialize the bus voltage ADC sensor
     *
     * Configures the ESP32 ADC channel, initializes calibration
     * characteristics (if available), and prepares internal resources.
     *
     * @param adcPin Arduino ADC pin connected to the voltage divider output
     * @return true if initialization succeeded
     */
    bool begin(uint8_t adcPin);

    /**
     * @brief Set voltage divider resistor values
     *
     * Updates the hardware voltage divider configuration and
     * recalculates the divider ratio:
     *
     *   dividerRatio = (Rtop + Rbottom) / Rbottom
     *
     * @param rTop     Top resistor value of the voltage divider (Ohms)
     * @param rBottom  Bottom resistor value of the voltage divider (Ohms)
     */
    void setResistorvalues(float rTop, float rBottom);

    /**
     * @brief Get the configured top resistor value
     * @return Top resistor value in Ohms
     */
    float getTopResistor() const { return _rTop; };

    /**
     * @brief Get the configured bottom resistor value
     * @return Bottom resistor value in Ohms
     */
    float getBottomResistor() const { return _rBottom; };

    /**
     * @brief Set the empirical voltage calibration factor
     *
     * This factor compensates ADC gain error and resistor tolerance
     * and is applied after ADC calibration and divider scaling.
     *
     * Typical values are close to 1.0.
     *
     * @param calibrationFactor Gain correction factor
     */
    bool setCalibrationFactor(float calibrationFactor);

    /**
     * @brief Get the current calibration factor
     * @return Calibration gain factor
     */
    float getCalibrationFactor() const { return _calibrationFactor; };

    /**
     * @brief Perform a non-blocking ADC sample and update the bus voltage
     *
     * This method should be called periodically from a task or main loop.
     * It performs:
     *  - Raw ADC sampling
     *  - ESP32 ADC calibration compensation
     *  - Voltage divider scaling
     *  - Empirical gain calibration
     *
     * The resulting calibrated bus voltage is stored internally.
     *
     * @return true  If a new sample was successfully acquired and stored
     * @return false If sampling failed or the sensor is not ready
     */
    bool update() override;

    /**
     * @brief Get the latest measured bus voltage
     *
     * Thread-safe accessor returning the most recent calibrated
     * bus voltage value.
     *
     * @return Bus voltage in volts
     */
    float getVoltage_V();

private:
    /**
     * @brief Convert raw ADC reading to bus voltage (pre-calibration)
     *
     * Applies ESP32 ADC calibration (if available) and
     * hardware voltage divider scaling.
     *
     * The empirical calibration factor is NOT applied here.
     *
     * @param raw Raw ADC reading
     * @return Bus voltage in volts before gain calibration
     */
    float adcToVoltage(uint32_t raw);

    /**
     * @brief Hardware voltage divider ratio
     *
     * Dimensionless ratio derived from resistor values:
     *
     *   dividerRatio = (Rtop + Rbottom) / Rbottom
     *
     * Used to scale the ADC pin voltage to the actual bus voltage.
     */
    float _dividerRatio = 1.1f;

    /**
     * @brief Empirical voltage calibration factor
     *
     * Gain correction applied after ADC calibration and divider scaling.
     * Compensates ADC gain error and resistor tolerance.
     */
    float _calibrationFactor = 1.0f;

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
     * @brief Latest measured and calibrated bus voltage
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
