#include <esp_log.h>
#include "current_ina226_sensor.h"

const char *CurrentIna226Sensor::TAG = "CurrentIna226";
bool CurrentIna226Sensor::_initialized = false;

CurrentIna226Sensor::CurrentIna226Sensor(uint8_t address) : _ina(address)
{
}

float CurrentIna226Sensor::getVoltage_V()
{
    if (!_initialized)
        return 0.;

    return (_ina.getBusVoltage());
};

float CurrentIna226Sensor::getShuntVoltage()
{
    if (!_initialized)
        return 0.;
    return (_ina.getShuntVoltage());
};

float CurrentIna226Sensor::getCurrent_A()
{
    if (!_initialized)
        return 0.;
    return (_ina.getCurrent());
};

float CurrentIna226Sensor::getPower_W()
{
    if (!_initialized)
        return 0.;
    return (_ina.getPower());
};

void CurrentIna226Sensor::calibrate()
{
    ESP_LOGI("calibrate_ina226", "# Calibrate INA226 Current Sensor");

    float bv = 0, cu = 0;
    for (int i = 0; i < 10; i++)
    {
        bv += _ina.getBusVoltage();
        cu += _ina.getCurrent_mA() / 1000.0;
        delay(150);
    }
    bv /= 10;
    cu /= 10;
    ESP_LOGI("calibrate_ina226", "Average Bus and Current values for use in Shunt Resistance, Bus Voltage and Current Zero Offset calibration:");
    bv = 0;
    for (int i = 0; i < 10; i++)
    {
        bv += _ina.getBusVoltage();
        delay(100);
    }
    bv /= 10;
    Serial.print("\nAverage of 10 Bus Voltage values = ");
    Serial.print(bv, 3);
    Serial.println("V");
    cu = 0;
    for (int i = 0; i < 10; i++)
    {
        cu += _ina.getCurrent_mA() / 1000.0;
        delay(100);
    }
    cu /= 10;
    Serial.print("Average of 10 Current values = ");
    Serial.print(cu, 3);
    Serial.println("mA");

    Serial.println("\nCALIBRATION VALUES TO USE:\t(DMM = Digital MultiMeter)");
    Serial.println("Step 5. Attach a power supply with voltage 5-10V to INA226 on VBUS/IN+ and GND pins, without any load.");
    Serial.print("\tcurrent_zero_offset_mA = ");
    Serial.print(current_zero_offset_mA + cu, 3);
    Serial.println("mA");
    if (cu > 5)
        Serial.println("********** NOTE: No resistive load needs to be present during current_zero_offset_mA calibration. **********");
    Serial.print("\tbus_V_scaling_e4 = ");
    Serial.print(bus_V_scaling_e4);
    Serial.print(" / ");
    Serial.print(bv, 3);
    Serial.println(" * (DMM Measured Bus Voltage)");
    Serial.println("Step 8. Set DMM in current measurement mode. Use a resistor that will generate around 50-100mA IOUT measurement between IN- and GND pins with DMM in series with load. Note current measured on DMM.");
    Serial.print("\tshunt = ");
    Serial.print(shunt, 4);
    Serial.print(" * ");
    Serial.print(cu, 3);
    Serial.println(" / (DMM Measured IOUT)");
    if (cu < 40)
        Serial.println("********** NOTE: IOUT needs to be more than 50mA for better shunt resistance calibration. **********");
}