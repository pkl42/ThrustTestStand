#include "ThermocoupleSensor.h"
#include <esp_log.h>

bool ThermocoupleSensor::begin(int8_t clkPin, int8_t csPin, int8_t soPin)
{
    if (isReady())
    {
        ESP_LOGI(TAG, "MAX31855 Thermocouple already initialized and ready.");
        return true;
    }

    ESP_LOGI(TAG, "Initialize MAX31855 Thermocouple ...");
    setState(SensorState::SENSOR_INITIALIZING);

    // Initialize MAX31855 with proper constructor
    if (csPin == -1)
        _tc = Adafruit_MAX31855(clkPin);
    else
        _tc = Adafruit_MAX31855(clkPin, csPin, soPin);

    // Test read
    float temp = _tc.readCelsius();
    if (isnan(temp))
    {
        setState(SensorState::SENSOR_ERROR);
        setDataValid(false);
        ESP_LOGE(TAG, "Failed to read thermocouple at init");
        return false;
    }

    _temperature = temp;
    setDataValid(true);
    setState(SensorState::SENSOR_READY);
    ESP_LOGI(TAG, "Thermocouple MAX31855 initialized.");
    return true;
}

bool ThermocoupleSensor::update()
{
    if (!isReady())
        return false;

    float temp = _tc.readCelsius();

    if (isnan(temp))
    {
        setDataValid(false);
        setState(SensorState::SENSOR_ERROR);

        uint8_t e = _tc.readError();
        if (e & MAX31855_FAULT_OPEN)
            ESP_LOGE(TAG, "FAULT: Thermocouple open");
        if (e & MAX31855_FAULT_SHORT_GND)
            ESP_LOGE(TAG, "FAULT: Short to GND");
        if (e & MAX31855_FAULT_SHORT_VCC)
            ESP_LOGE(TAG, "FAULT: Short to VCC");

        return false;
    }

    _temperature = temp;
    setDataValid(true);
    ++_updateCount; // make sure _updateCount exists in header or remove
    return true;
}

float ThermocoupleSensor::getTemperatureInCelsius() const
{
    return _temperature;
}
