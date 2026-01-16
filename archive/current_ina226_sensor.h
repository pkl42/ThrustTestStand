#ifndef CURRENT_INA226_SENSOR_H
#define CURRENT_INA226_SENSOR_H
#include <INA226.h>

#define I2C_ADDRESS 0x40

class CurrentIna226Sensor
{
public:
    CurrentIna226Sensor(const uint8_t address);
    bool begin(float calibration);

    void calibrate();

    float getVoltage_V();
    float getShuntVoltage();
    float getCurrent_A();
    float getPower_W();

private:
    static const char *TAG;
    INA226 _ina;

    float cal = (481.15 / 290.0);
    float shunt = 0.002 * (481.15 / 290.0);   /* shunt (Shunt Resistance in Ohms). Lower shunt gives higher accuracy but lower current measurement range. Recommended value 0.020 Ohm. Min 0.001 Ohm */
    float INA266_max_current = 0.081 / shunt; /* INA226 max current depends on shunt restistor: 0.081 * shunt  */
    float current_LSB_mA = 0.5;               /* current_LSB_mA (Current Least Significant Bit in milli Amperes). Recommended values: 0.050, 0.100, 0.250, 0.500, 1, 2, 2.5 (in milli Ampere units) */
    float current_zero_offset_mA = -23.850;   // -39.650*cal;    /* current_zero_offset_mA (Current Zero Offset in milli Amperes, default = 0) | Needs to be updated along with shunt!! */
    uint16_t bus_V_scaling_e4 = 10000;

    static bool _initialized;
};

#endif // CURRENT_INA266_SENSOR_H