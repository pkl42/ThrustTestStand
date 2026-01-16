#ifndef PINOUT_H
#define PINOUT_H
#include <IPAddress.h>

#define THRUSTSTAND_APP_NAME "Motor Thrust Test Stand"
#define THRUSTSTAND_APP_VERSION "v1.5.0"
#define THRUSTSTAND_CSV_VERSION "1.1"

// #define THRUSTSTAND_GIT_HASH        "a9c4e2f"
#define THRUSTSTAND_BUILD_DATE __DATE__ " " __TIME__

/*
ESP32 S3 Pins
https://www.wemos.cc/en/latest/s3/s3.html
---+------+----+-----+-----+-----------+---------------------------
No.| GPIO | IO | RTC | ADC | Default   | Function
---+------+----+-----+-----+-----------+---------------------------
   |   0* | IO |   0 |     | Boot      | Button
   |   1  | IO |   1 | 1_0 |           | CURRENT_SENSOR_PIN ACS758
   |   2  | IO |   2 | 1_1 |           | Voltage
   |   3* | IO |   3 | 1_2 |           |
   |   4  | IO |   4 | 1_3 |           | HX711-DOUT1
   |   5  | IO |   5 | 1_4 |           | HX711-CLK1
   |   6  | IO |   6 | 1_5 |           | HX711-DOUT2
   |   7  | IO |   7 | 1_6 |           | HX711-CLK2
   |   8  | IO |   8 | 1_7 |           |
   |   9  | IO |   9 | 1_8 |           |
   |  10  | IO |  10 | 1_9 | SPI-SS    |
   |  11  | IO |  11 | 2_0 | SPI-MOSI  |
   |  12  | IO |  12 | 2_1 | SPI-SCK   |
   |  13  | IO |  13 | 2_2 | SPI-MISO  |
   |  14  | IO |  14 | 2_3 |           | ESC-PWM
   |  15  | IO |  15 | 2_4 |           | HX711-DOUT3
   |  16  | IO |  16 | 2_5 |           | HX711-CLK3
   |  17  | IO |  17 | 2_6 |           | MAX31855_CS_PIN
   |  18  | IO |  18 | 2_7 |           |
   |  19  | IO |  19 | 2_8 | USB/JTAG  |
   |  20  | IO |  20 | 2_9 | USB/JTAG  |
   |  21  | IO |  21 |     |           |
   |  38  | IO |     |     |           | RGB_BUILTIN_LED
   |  39  | IO |     |     |           |
   |  40  | IO |     |     |           |
   |  41  | IO |     |     | I2C_SCL   |
   |  42  | IO |     |     | I2C_SDA   | CAGE_SWITCH_PIN
   |  43  | IO |     |     | UART_TX0  |
   |  44  | IO |     |     | UART_RX0  |
   |  45* | IO |     |     |           |
   |  46* | IO |     |     |           |
   |  47  | IO |     |     |           | ESTOP_PIN
   |  48  | IO |     |     |           |
---+------+----+-----+-----+-----------+---------------------------
* Strapping pins: IO0, IO3, IO45, IO46
*/

// if this clause does not work, but the builtin RGB is working
// just add this into platformio.ini
//
// build_flags =
//     -DHAS_RGB_LED
//     -DRGB_BUILTIN=48
// There is currently a bug in the ESP32-S3 neopixelWrite() code. This will produce the
// following errors and the RGB LED will not light up.:
//  E (19) rmt: rmt_set_gpio(526): RMT GPIO ERROR
//  E (19) rmt: rmt_config(686): set gpio for RMT driver failed
//  ==> Remove the #ifdef RGB_BUILTIN logic in the neopixelWrite() code.
// ==> or i choose RGB_BUILTIN_LED

// #if defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(RGB_BUILTIN)
// #define RGB_BUILTIN 48
// #define HAS_RGB_LED 1
// #endif

#define RGB_BUILTIN_LED 38
#define HAS_RGB_LED 1

#define HX711_DOUT_1_PIN 4 // mcu > HX711 no 1 dout pin 5kg
#define HX711_SCK_1_PIN 5  // mcu > HX711 no 1 sck pin 5kg

#define HX711_DOUT_2_PIN 6 // mcu > HX711 no 2 dout pin 2kg-1
#define HX711_SCK_2_PIN 7  // mcu > HX711 no 2 sck pin 2kg-1

#define HX711_DOUT_3_PIN 15 // mcu > HX711 no 3 dout pin 2kg-2
#define HX711_SCK_3_PIN 16  // mcu > HX711 no 3 sck pin 2kg-2

#define MOTOR_ESC_PIN 14

#define RPM_SENSOR_PIN 13 // GPIO pin connected to the optical sensor

#define CURRENT_SENSOR_PIN 1 // GPIO pin for current sensor ACS758

#define VOLTAGE_SENSOR_PIN 2 // GPIO pin for voltage sensor

#define MAX31855_CS_PIN 17

#define ESTOP_PIN 47

#define CAGE_SWITCH_PIN 42

// microsecond delay after writing sck pin high or low. This delay could be required for faster mcu's.
// So far the only mcu reported to need this delay is the ESP32 (issue #35), both the Arduino Due and ESP8266 seems to run fine without it.
// Change the value to '1' to enable the delay.
#define SCK_DELAY 0 // default value: 0

// if you have some other time consuming (>60μs) interrupt routines that trigger while the sck pin is high, this could unintentionally set the HX711 into "power down" mode
// if required you can change the value to '1' to disable interrupts when writing to the sck pin.
#define SCK_DISABLE_INTERRUPTS 0 // default value: 0

// ==============================================
// Installed Sensors
// ==============================================

const bool THRUST_SENSOR_INSTALLED = true;
const bool TORQUE_SENSOR_INSTALLED = true;
const bool CURRENT_SENSOR_INSTALLED = true;
const bool VOLTAGE_SENSOR_INSTALLED = true;

const bool TEMPERATURE_SENSOR_INSTALLED = false;
const bool RPM_SENSOR_INSTALLED = true;

// ==============================================
// Web Access Point
// ==============================================
// AP configuration
inline const char *AP_SSID = "ThrustStandAP";
inline const char *AP_PASSWORD = "4711"; // leave empty for open AP

inline const IPAddress AP_IP(192, 168, 7, 1);
inline const IPAddress AP_GATEWAY(192, 168, 7, 1);
inline const IPAddress AP_SUBNET(255, 255, 255, 0);
#endif // PINOUT_H