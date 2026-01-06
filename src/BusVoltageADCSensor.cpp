#include "BusVoltageADCSensor.h"
#include <esp_log.h>

BusVoltageADC::BusVoltageADC() = default;

BusVoltageADC::~BusVoltageADC()
{
    if (_mutex)
    {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}

bool BusVoltageADC::begin(uint8_t adcPin)
{
    if (isReady())
    {
        // setConfig(config);
        ESP_LOGI(TAG, "BusVoltageADC Sensor already initialized and ready.");
        return true;
    }

    ESP_LOGI(TAG, "Initialize BusVoltageADC Sensor ...");
    setState(SensorState::SENSOR_INITIALIZING);

    _pin = adcPin;

    analogReadResolution(12);
    analogSetPinAttenuation(_pin, ADC_11db);

    _adcChannel = (adc1_channel_t)digitalPinToAnalogChannel(_pin);

    // ADC calibration
    esp_adc_cal_value_t calType =
        esp_adc_cal_characterize(ADC_UNIT_1,
                                 ADC_ATTEN_11db, // Use new enum instead of deprecated macro
                                 ADC_WIDTH_BIT_12,
                                 1100,
                                 &_adcChars);
    _adcCalibrated = (calType != ESP_ADC_CAL_VAL_NOT_SUPPORTED);

    ESP_LOGI(TAG, "ADC calibration: %s",
             _adcCalibrated ? "enabled" : "not supported");

    _mutex = xSemaphoreCreateMutex();
    if (!_mutex)
    {
        setState(SensorState::SENSOR_ERROR);
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }

    setState(SensorState::SENSOR_READY);
    ESP_LOGI(TAG, "BusVoltageADC initialized on ADC pin %u", _pin);
    return true;
}

bool BusVoltageADC::update()
{
    if (!isReady() || !_mutex)
        return false;

    uint32_t raw = adc1_get_raw(_adcChannel);
    float voltage = adcToVoltage(raw);

    xSemaphoreTake(_mutex, 0);
    _voltage = voltage;
    setDataValid(true);
    xSemaphoreGive(_mutex);
    return true;
}

float BusVoltageADC::getVoltage_V()
{
    if (!_mutex || !isReady())
        return 0.0f;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    float v = _voltage;
    xSemaphoreGive(_mutex);

    return v;
}

float BusVoltageADC::adcToVoltage(uint32_t raw)
{
    float v;
    if (_adcCalibrated)
    {
        uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &_adcChars);
        v = (mv / 1000.0f) * _scale;
    }
    else
    {
        v = ((raw / 4095.0f) * 3.3f) * _scale;
    }
    return v;
}

void BusVoltageADC::setResistorvalues(float rTop, float rBottom)
{
    _rTop = rTop;
    _rBottom = rBottom;
    _scale = (_rTop + _rBottom) / _rBottom;
}
