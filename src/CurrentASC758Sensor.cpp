#include "CurrentACS758Sensor.h"
#include <esp_log.h>

CurrentACS758::CurrentACS758() = default;

CurrentACS758::~CurrentACS758()
{
    if (_mutex)
    {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}

bool CurrentACS758::begin(uint8_t adcPin, float sensorVcc)
{
    if (isReady())
    {
        ESP_LOGI(TAG, "ACS758 already initialized.");
        return true;
    }

    ESP_LOGI(TAG, "Initializing ACS758 current sensor...");
    setState(SensorState::SENSOR_INITIALIZING);

    _pin = adcPin;
    _sensorVcc = sensorVcc;

    analogReadResolution(12);
    analogSetPinAttenuation(_pin, ADC_11db);

    // Map Arduino pin → ADC1 channel
    _adcChannel = (adc1_channel_t)digitalPinToAnalogChannel(_pin);

    // ADC calibration (ESP32)
    esp_adc_cal_value_t calType =
        esp_adc_cal_characterize(ADC_UNIT_1,
                                 ADC_ATTEN_11db,
                                 ADC_WIDTH_BIT_12,
                                 1100, // default internal reference
                                 &_adcChars);

    _adcCalibrated = (calType != ESP_ADC_CAL_VAL_NOT_SUPPORTED);

    ESP_LOGI(TAG, "ADC calibration: %s",
             _adcCalibrated ? "enabled" : "not supported");

    _mutex = xSemaphoreCreateMutex();
    if (!_mutex)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        setState(SensorState::SENSOR_ERROR);
        return false;
    }

    // Default ratiometric zero offset (ACS758 output = Vcc / 2 @ 0 A)
    _zeroOffset = _sensorVcc * 0.5f;

    setState(SensorState::SENSOR_READY);
    ESP_LOGI(TAG, "ACS758 initialized on ADC pin %u", _pin);

    return true;
}

bool CurrentACS758::update()
{
    if (!isReady() || !_mutex)
        return false;

    // Non-blocking mutex acquisition
    if (xSemaphoreTake(_mutex, 0) != pdTRUE)
        return false;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < _avgSamples; i++)
    {
        sum += adc1_get_raw(_adcChannel);
    }

    float voltage = adcToVoltage(sum / _avgSamples);

    // Initialize filter on first valid sample
    if (!hasValidData())
        _filteredVoltage = voltage;
    else
        _filteredVoltage = applyFiltering(voltage);

    setDataValid(true);

    xSemaphoreGive(_mutex);
    return true;
}

void CurrentACS758::setSensitivity(float voltsPerAmp)
{
    _sensitivity = voltsPerAmp;
}

void CurrentACS758::setSensitivityFromVcc(float mVperAmp_at5V)
{
    _sensitivity = (mVperAmp_at5V / 1000.0f) * (_sensorVcc / 5.0f);
}

#include <cmath> // for isnan

void CurrentACS758::calibrateSensitivityByCurrent(
    float actualCurrent_A,
    float measuredCurrent_A) // optional
{
    if (actualCurrent_A <= 0.0f)
        return;

    // auto-read if not provided
    if (std::isnan(measuredCurrent_A))
    {
        measuredCurrent_A = getCurrent_A();
    }

    // avoid division by zero
    if (actualCurrent_A == 0.0f)
        return;

    _sensitivity *= (measuredCurrent_A / actualCurrent_A);

    ESP_LOGI(TAG,
             "Calibrate: measured=%.3f A, actual=%.3f A, newSens=%.5f",
             measuredCurrent_A, actualCurrent_A, _sensitivity);
}

float CurrentACS758::adcToVoltage(uint32_t raw)
{
    if (_adcCalibrated)
    {
        uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &_adcChars);
        return mv / 1000.0f;
    }
    else
    {
        // Fallback (not ideal, but safe)
        return (raw / 4095.0f) * 3.3f;
    }
}

float CurrentACS758::applyFiltering(float v)
{
    _filteredVoltage =
        (_iirAlpha * v) + ((1.0f - _iirAlpha) * _filteredVoltage);

    return _filteredVoltage;
}

float CurrentACS758::getVoltage_V()
{
    if (!isReady() || !_mutex)
    {
        return 0.0f;
    }

    xSemaphoreTake(_mutex, portMAX_DELAY);
    float v = _filteredVoltage;
    xSemaphoreGive(_mutex);

    return v;
}

float CurrentACS758::getCurrent_A()
{
    float current = (getVoltage_V() - _zeroOffset) / _sensitivity;
    return isnan(current) ? 0.0f : current;
}

void CurrentACS758::setMovingAverage(uint8_t samples)
{
    _avgSamples = max<uint8_t>(1, samples);
}

void CurrentACS758::setIIR(float alpha)
{
    _iirAlpha = constrain(alpha, 0.0f, 1.0f);
}

bool CurrentACS758::tare(uint16_t samples)
{
    ESP_LOGI(TAG, "ACS758 calibrating zero offset...");
    if (!isReady() || !_mutex)
    {
        ESP_LOGE(TAG, "ACS758 sensor not ready!");
        return false;
    }

    xSemaphoreTake(_mutex, portMAX_DELAY);

    uint64_t sum = 0;
    for (uint16_t i = 0; i < samples; i++)
    {
        sum += adc1_get_raw(_adcChannel);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    float avgRaw = sum / (float)samples;
    _zeroOffset = adcToVoltage(avgRaw);

    xSemaphoreGive(_mutex);
    ESP_LOGI(TAG, "Zero calibrated: %.3f V", _zeroOffset);
    return true;
}
