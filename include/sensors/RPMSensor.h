/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef RPM_SENSOR_H
#define RPM_SENSOR_H

#include <Arduino.h>
#include "BaseSensor.h"
#include "driver/pcnt.h"

/**
 * @enum RpmResponseMode
 * @brief Controls measurement responsiveness vs stability.
 *
 * Defines the sampling interval used for RPM calculation.
 *
 * - FAST: Low latency, higher noise
 * - BALANCED: Trade-off between latency and stability
 * - STABLE: High stability, slower response
 */
enum class RpmResponseMode : uint8_t
{
    FAST = 0,
    BALANCED = 1,
    STABLE = 2
};

inline const char *rpmResponseToString(RpmResponseMode mode)
{
    switch (mode)
    {
    case RpmResponseMode::FAST:
        return "Fast";
    case RpmResponseMode::BALANCED:
        return "Balanced";
    case RpmResponseMode::STABLE:
        return "Stable";
    default:
        return "Unknown";
    }
}

/**
 * @brief Convert response mode to sampling interval.
 *
 * @return Sample time in milliseconds
 */
inline uint16_t rpmResponseToSampleTime(RpmResponseMode mode)
{
    switch (mode)
    {
    case RpmResponseMode::FAST:
        return 50;
    case RpmResponseMode::BALANCED:
        return 100;
    case RpmResponseMode::STABLE:
        return 200;
    default:
        return 100;
    }
}

/**
 * @enum RpmFilterLevel
 * @brief Hardware glitch filter strength.
 *
 * Configures PCNT input filtering to suppress noise pulses.
 *
 * - WEAK: Minimal filtering, high responsiveness
 * - NORMAL: Balanced filtering
 * - STRONG: Maximum noise suppression
 */
enum class RpmFilterLevel : uint8_t
{
    WEAK = 0,   // allows more noise, better responsiveness
    NORMAL = 1, // balanced
    STRONG = 2  // maximum stability
};

inline const char *rpmFilterToString(RpmFilterLevel level)
{
    switch (level)
    {
    case RpmFilterLevel::WEAK:
        return "Weak";
    case RpmFilterLevel::NORMAL:
        return "Normal";
    case RpmFilterLevel::STRONG:
        return "Strong";
    default:
        return "Unknown";
    }
}

/**
 * @brief Convert filter level to PCNT filter threshold.
 *
 * @return Filter threshold in APB clock cycles (max 1023)
 *
 * @note
 * Values are clamped to hardware limits.
 */
inline uint16_t rpmFilterToCycles(RpmFilterLevel level)
{
    constexpr uint32_t APB_MHZ = 80;

    auto usToCycles = [&](uint32_t us) -> uint16_t
    {
        uint32_t cycles = us * APB_MHZ;
        return (cycles > 1023) ? 1023 : (uint16_t)cycles;
    };

    switch (level)
    {
    case RpmFilterLevel::WEAK:
        return usToCycles(5); // 400 cycles
    case RpmFilterLevel::NORMAL:
        return usToCycles(10); // 800 cycles

    case RpmFilterLevel::STRONG:
        return usToCycles(12); // 960 cycles (safe max-ish)
    default:
        return usToCycles(10);
    }
}

/**
 * @enum RpmSmoothing
 * @brief Software smoothing level for RPM output.
 *
 * Controls exponential low-pass filtering strength.
 *
 * - RESPONSIVE: Fast changes, less smoothing
 * - BALANCED: Moderate smoothing
 * - SMOOTH: Stable output, slower response
 */
enum class RpmSmoothing : uint8_t
{
    RESPONSIVE = 0,
    BALANCED = 1,
    SMOOTH = 2
};

inline const char *rpmSmoothingToString(RpmSmoothing s)
{
    switch (s)
    {
    case RpmSmoothing::RESPONSIVE:
        return "Responsive";
    case RpmSmoothing::BALANCED:
        return "Balanced";
    case RpmSmoothing::SMOOTH:
        return "Smooth";
    default:
        return "Unknown";
    }
}

/**
 * @brief Convert smoothing mode to filter coefficient.
 *
 * @return Alpha value for exponential smoothing (0..1)
 */
inline float rpmSmoothingToAlpha(RpmSmoothing s)
{
    switch (s)
    {
    case RpmSmoothing::RESPONSIVE:
        return 0.6f;
    case RpmSmoothing::BALANCED:
        return 0.3f;
    case RpmSmoothing::SMOOTH:
        return 0.1f;
    default:
        return 0.3f;
    }
}

/**
 * @struct RpmUiConfig
 * @brief User-facing configuration for RPM measurement.
 *
 * Provides intuitive parameters that are converted into
 * hardware and runtime configuration.
 *
 * @section fields Fields
 * - pulsesPerRevolution: Sensor-specific pulse count
 * - responseMode: Measurement speed vs stability
 * - filterLevel: Hardware noise filtering
 * - smoothing: Software filtering
 *
 * @note
 * This configuration is converted internally to @ref RpmSensorConfig.
 */
struct RpmUiConfig
{
    uint8_t pulsesPerRevolution = 1;
    RpmResponseMode responseMode = RpmResponseMode::BALANCED;
    RpmFilterLevel filterLevel = RpmFilterLevel::NORMAL;
    RpmSmoothing smoothing = RpmSmoothing::BALANCED;
};

inline bool operator==(const RpmUiConfig &a, const RpmUiConfig &b)
{
    return a.pulsesPerRevolution == b.pulsesPerRevolution &&
           a.responseMode == b.responseMode &&
           a.filterLevel == b.filterLevel &&
           a.smoothing == b.smoothing;
}

/**
 * @class RpmSensor
 * @brief Hardware-based RPM sensor using ESP32 PCNT peripheral.
 *
 * Measures rotational speed by counting input pulses over time.
 * Designed for hall sensors, optical sensors, or ESC telemetry signals.
 *
 * @section rpm_model Measurement Model
 *
 * RPM is calculated by:
 *
 *   RPM = (pulse_count / pulses_per_revolution) * (60000 / sample_time_ms)
 *
 * Where:
 * - pulse_count = pulses measured within the sampling window
 * - pulses_per_revolution = sensor-specific configuration
 * - sample_time_ms = measurement interval
 *
 *
 * @section rpm_pipeline Signal Processing Pipeline
 *
 * The measured signal is processed in multiple stages:
 *
 * 1. Hardware pulse counting (PCNT)
 * 2. Glitch filtering (PCNT filter)
 * 3. Time-window sampling
 * 4. Low-pass filtering (exponential smoothing)
 *
 *
 * @section rpm_response Response vs Stability
 *
 * Behavior is controlled via @ref RpmUiConfig:
 *
 * - Response mode → sampling interval (latency vs resolution)
 * - Filter level → hardware glitch filtering
 * - Smoothing → software low-pass filtering
 *
 *
 * @section rpm_timeout Timeout Behavior
 *
 * - If no pulses are detected for a configurable duration,
 *   RPM is forced to zero
 * - Prevents stale values when motor stops
 *
 *
 * @section rpm_validity Data Validity
 *
 * - getRPM() is only valid if hasValidData() returns true
 * - Invalid data may occur during initialization or signal loss
 *
 *
 * @section rpm_threading Threading Model
 *
 * - update() must be called periodically (non-ISR context)
 * - Uses hardware PCNT peripheral internally
 *
 *
 * @note
 * This class abstracts hardware pulse counting and provides a stable,
 * filtered RPM signal suitable for control and logging.
 */
class RpmSensor : public BaseSensor
{
public:
    RpmSensor() = default;
    ~RpmSensor();

    /**
     * @brief Initialize the RPM sensor.
     *
     * Configures the PCNT peripheral and applies the given UI configuration.
     *
     * @param pin Input GPIO for pulse signal
     * @param config User-facing configuration
     *
     * @return true if initialization succeeded, false otherwise
     *
     * @note
     * - Must be called before update()
     * - Transitions sensor to READY state on success
     */
    bool begin(uint8_t pin,
               const RpmUiConfig &config);

    /**
     * @brief Perform a measurement update cycle.
     *
     * - Reads pulse count from PCNT
     * - Computes RPM based on sampling window
     * - Applies smoothing filter
     * - Updates data validity
     *
     * @return true if update succeeded, false otherwise
     */
    bool update() override;

    /**
     * @brief Get the current filtered RPM value.
     *
     * @return Rotational speed in revolutions per minute
     *
     * @note
     * - Only valid if hasValidData() returns true
     * - Value is low-pass filtered
     */
    float getRPM() const;

    /**
     * @brief Get raw pulse count from PCNT.
     */

    uint32_t getPulseCount(bool resetCounter = true);

    /**
     * @brief Reset internal state and counters.
     *
     * Does not reinitialize hardware.
     */
    void reset();
    /**
     * @brief Deinitialize the sensor and release hardware resources.
     */
    void deinit();

    /**
     * @brief Get current user configuration.
     */
    RpmUiConfig getUIConfig() const { return _uiConfig; };

    /**
     * @brief Set user configuration.
     *
     * Applies new configuration and updates runtime parameters.
     *
     * @param uiConfig New configuration
     * @return true if applied successfully
     *
     * @note
     * May reconfigure PCNT filter and timing parameters.
     */
    bool setUIConfig(const RpmUiConfig &uiConfig);

private:
    bool initPCNT();
    void deinitPCNT();
    void applyPcntFilter();

    static const char *TAG;

    /* ========= Configuration ========= */

    struct RpmSensorConfig
    {
        uint8_t pulsesPerRevolution = 1; // Pulses per full revolution
        uint16_t sampleTimeMs = 100;     // Measurement window
        uint16_t timeoutMs = 2000;       // No pulses → RPM = 0
        uint16_t filterCycles = 1000;    // PCNT glitch filter (APB cycles)
        float alpha = 0.3f;              // Low-pass filter (0..1)
    };

    RpmSensorConfig convertUIConfigToRuntime(const RpmUiConfig &ui);
    RpmSensorConfig _config;
    RpmUiConfig _uiConfig;

    uint8_t _pin = 0;

    /* ========= PCNT ========= */

    pcnt_unit_t _pcntUnit = PCNT_UNIT_0;

    /* ========= Timing ========= */

    uint32_t _lastSampleTimeMs = 0;
    uint32_t _lastPulseTimeMs = 0;

    /* ========= State ========= */

    int16_t _lastCount = 0;
    float _rpm = 0.0f;
    float _rpmFiltered = 0.0f;
};

#endif // RPM_SENSOR_H