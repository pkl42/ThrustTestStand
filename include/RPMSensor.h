#ifndef RPM_SENSOR_H
#define RPM_SENSOR_H

#include <Arduino.h>
#include "BaseSensor.h"

/**
 * @brief Configuration parameters for the RPM sensor
 */
struct RpmSensorConfig
{
    uint8_t pulsesPerRevolution = 1; // Pulses per full revolution
    uint16_t debounceUs = 500;       // Minimum pulse spacing (µs)
    uint16_t timeoutMs = 2000;       // No pulse → RPM = 0
    float outlierThreshold = 0.2f;   // Relative deviation rejection
    uint8_t periodBufferSize = 8;    // Raw period buffer size
    uint8_t averageBufferSize = 32;  // Filtered average buffer size
};

/**
 * @class RpmSensor
 * @brief Interrupt-driven RPM sensor using pulse timing
 */
class RpmSensor : public BaseSensor
{
public:
    RpmSensor() = default;
    ~RpmSensor();

    /**
     * @brief Initialize the RPM sensor
     *
     * @param pin GPIO pin connected to the sensor
     * @param config Sensor configuration
     * @return true if initialization succeeded
     */
    bool begin(uint8_t pin,
               const RpmSensorConfig &config);

    // void setConfig(const RpmSensorConfig &config);

    /**
     * @brief Update RPM calculation (call periodically)
     *
     * @return true if a valid RPM value is available
     */
    bool update() override;

    /**
     * @brief Get the current RPM value
     */
    float getRPM() const;

    /**
     * @brief Get pulse count since last read
     *
     * @param resetCounter Reset counter after reading
     * @return Number of pulses detected
     */
    uint32_t getPulseCount(bool resetCounter = true);

private:
    /* ========= ISR ========= */

    static void IRAM_ATTR pulseCounter();

    /* ========= Instance-only processing ========= */

    bool processNewInterval(uint32_t interval);

private:
    static const char *TAG;

    /* ========= ISR-shared (must be static + volatile) ========= */

    static RpmSensor *_instance;               // ISR → object bridge
    static volatile uint32_t _pulseCount;      // Total pulse count
    static volatile uint32_t _lastPulseTimeUs; // Timestamp of last pulse
    static volatile uint32_t _pulseIntervalUs; // Last pulse interval

    /* ========= Instance configuration ========= */

    RpmSensorConfig _config;
    uint8_t _pin = 0;

    /* ========= Filtering buffers (instance-owned) ========= */

    uint32_t *_periodBuffer = nullptr;
    uint32_t *_averageBuffer = nullptr;

    uint8_t _periodBufferIndex = 0;
    uint8_t _averageBufferIndex = 0;

    uint32_t _thresholdPeriod = 0;

    /* ========= Output ========= */

    float _rpm = 0.0f;
};

#endif // RPM_SENSOR_H
