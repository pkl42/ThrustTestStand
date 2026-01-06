#ifndef CURRENT_ACS758_SENSOR_H
#define CURRENT_ACS758_SENSOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#include "BaseSensor.h"

/**
 * @class CurrentACS758
 * @brief Non-blocking, calibrated current sensor driver for ACS758 (ESP32)
 *
 * This class implements a task-safe, non-blocking driver for the ACS758
 * Hall-effect current sensor using the ESP32 ADC.
 *
 * Features:
 *  - ESP32 ADC calibration compensation
 *  - Moving average + IIR filtering
 *  - Non-blocking update() sampling
 *  - Thread-safe access using FreeRTOS mutex
 *  - Designed for motor thrust / power measurement systems
 */
class CurrentACS758 : public BaseSensor
{
public:
    /**
     * @brief Construct a new CurrentACS758 sensor instance
     */
    CurrentACS758();

    /**
     * @brief Destroy the CurrentACS758 sensor instance
     */
    ~CurrentACS758();

    /**
     * @brief Initialize the ACS758 current sensor
     *
     * @param adcPin     Arduino ADC pin connected to ACS758 output
     * @param sensorVcc  Supply voltage of the ACS758 sensor (default 3.3 V)
     * @return true if initialization succeeded
     */
    bool begin(uint8_t adcPin, float sensorVcc = 3.3f);

    /**
     * @brief Perform a non-blocking ADC sample and update filtered voltage
     *
     * This method should be called periodically (e.g. from a task or loop).
     * It performs ADC sampling, calibration compensation, and filtering.
     *
     * @return true  If a new sample was successfully acquired and the internal
     *               voltage(representing current) value was updated with valid data.
     * @return false If the sample could not be completed or was invalid
     *               (e.g. ADC not ready, conversion error, out-of-range reading).
     */
    bool update() override;

    /**
     * @brief Calibrate the zero-current offset
     *
     * The sensor input must have zero current flowing during calibration.
     *
     * @param samples Number of ADC samples used for averaging
     */
    void calibrateZero(uint16_t samples = 500);

    /**
     * @brief Manually set the sensor sensitivity
     *
     * @param voltsPerAmp Effective sensor sensitivity in volts per ampere
     */
    void setSensitivity(float voltsPerAmp);

    /**
     * @brief Get the Sensitivity object
     *
     * @return float
     */
    float getSensitivity() const { return (_sensitivity); };

    /**
     * @brief Set sensor sensitivity using datasheet value at 5 V
     *
     * Converts a datasheet mV/A value specified at 5 V supply
     * to the effective sensitivity at the actual sensor Vcc.
     *
     * @param mVperAmp_at5V Sensitivity in mV/A specified at 5 V
     */
    void setSensitivityFromVcc(float mVperAmp_at5V);

    /**
     * @brief Set the number of samples for the moving average filter
     *
     * @param samples Number of samples (minimum 1)
     */
    void setMovingAverage(uint8_t samples);

    /**
     * @brief Set the IIR filter coefficient
     *
     * @param alpha IIR coefficient (0.0 = slow, 1.0 = no filtering)
     */
    void setIIR(float alpha);

    /**
     * @brief Get the latest filtered sensor output voltage
     *
     * @return Filtered sensor voltage in volts
     */
    float getVoltage_V();

    /**
     * @brief Get the calculated current
     *
     * @return Current in amperes
     */
    float getCurrent_A();

private:
    /**
     * @brief Supply voltage of the ACS758 sensor
     *
     * Used for ratiometric offset and sensitivity scaling.
     */
    float _sensorVcc = 3.3f;

    /**
     * @brief Convert raw ADC reading to voltage
     *
     * Applies ESP32 ADC calibration if available.
     *
     * @param raw Raw ADC reading
     * @return Voltage in volts
     */
    float adcToVoltage(uint32_t raw);

    /**
     * @brief Apply IIR filtering to voltage sample
     *
     * @param v Input voltage
     * @return Filtered voltage
     */
    float applyFiltering(float v);

    /**
     * @brief ADC pin connected to the ACS758 output
     */
    uint8_t _pin = 0;

    /**
     * @brief ESP32 ADC1 channel corresponding to the pin
     */
    adc1_channel_t _adcChannel;

    /**
     * @brief Sensor sensitivity (V/A) at current sensor supply voltage
     */
    float _sensitivity = 0.0264f;

    /**
     * @brief Zero-current voltage offset (typically Vcc / 2)
     */
    float _zeroOffset = 0.0f;

    /**
     * @brief Number of samples used for moving average
     */
    uint8_t _avgSamples = 8;

    /**
     * @brief IIR filter coefficient
     */
    float _iirAlpha = 0.2f;

    /**
     * @brief Latest filtered sensor voltage
     */
    float _filteredVoltage = 0.0f;

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
    static constexpr const char *TAG = "ACS758";
};

#endif // CURRENT_ACS758_SENSOR_H
