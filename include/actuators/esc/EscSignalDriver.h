/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ESC_SIGNAL_DRIVER_H
#define ESC_SIGNAL_DRIVER_H

/**
 * @page esc_system ESC Control System
 *
 * @section overview Overview
 * The ESC control system provides a unified interface for driving
 * different ESC protocols.
 *
 * @section architecture Architecture
 * - EscSignalDriver: abstract interface
 * - PwmEscDriver: PWM implementation (servo-style)
 * - DShotEscDriver: digital protocol implementation
 *
 * @section mapping Throttle Mapping
 * All drivers accept a normalized throttle input:
 * - Range: 0–100%
 *
 * Internally mapped to:
 * - PWM: pulse width (µs)
 * - DShot: digital value (48–2047)
 *
 * @section configuration Configuration
 * ESCConfig provides protocol-independent configuration.
 *
 * @section safety Safety Considerations
 * - No implicit arming/disarming logic
 * - No ramp limiting
 * - Must be handled by higher-level system (e.g. ThrustStand)
 */

/**
 * @enum EscDriverType
 * @brief Identifies the ESC signal protocol implementation.
 *
 * Used for:
 * - Driver selection at runtime
 * - Logging and telemetry output
 * - Mapping to concrete driver implementations
 *
 * @see EscSignalDriver
 */
enum class EscDriverType : uint8_t
{
    ESC_DRIVER_NONE = 0, ///< No driver selected / uninitialized
    ESC_DRIVER_PWM = 1,  ///< Standard servo-style PWM
    ESC_DRIVER_DSHOT300 = 2,
    ESC_DRIVER_DSHOT600 = 3,
    ESC_DRIVER_DSHOT1200 = 4
};

/**
 * @brief Convert ESC driver type to human-readable label.
 *
 * @param driver ESC driver type
 * @return Constant string, never null.
 */
inline const char *escDriverToString(EscDriverType driver)
{
    switch (driver)
    {
    case EscDriverType::ESC_DRIVER_NONE:
        return "None";
    case EscDriverType::ESC_DRIVER_PWM:
        return "PWM";
    case EscDriverType::ESC_DRIVER_DSHOT1200:
        return "D-Shot 1200";
    case EscDriverType::ESC_DRIVER_DSHOT300:
        return "D-Shot 300";
    case EscDriverType::ESC_DRIVER_DSHOT600:
        return "D-Shot 600";
    default:
        return "Unknown ESC Driver";
    }
}

/**
 * @struct ESCConfig
 * @brief Common configuration for ESC drivers.
 *
 * Provides a protocol-independent configuration layer.
 * Not all parameters are used by all drivers.
 *
 * @note
 * - PWM drivers use minPulseUs / maxPulseUs.
 * - DShot drivers may ignore pulse settings.
 *
 * @warning
 * minSpinPercentage is applied on top of protocol-specific limits.
 */
struct ESCConfig
{
    uint16_t minPulseUs = 1000;
    uint16_t maxPulseUs = 2000;
    uint8_t minSpinPercentage = 0;
};

/**
 * @class EscSignalDriver
 * @brief Abstract interface for ESC signal generation.
 *
 * Defines a unified interface for controlling ESCs independent of
 * the underlying signal protocol (e.g. PWM or DShot).
 *
 * Implementations are responsible for translating a normalized
 * throttle value [0–100%] into protocol-specific signals.
 *
 * @section model Control Model
 * The control flow is:
 * 1. init()        → allocate hardware resources
 * 2. setConfig()   → apply limits and behavior
 * 3. writeThrottle() → send throttle commands
 * 4. writeIdle()   → maintain idle/armed state
 * 5. deinit()      → release resources
 *
 * @section units Units and Mapping
 * - Input: throttle in percent [0.0 – 100.0]
 * - Output:
 *   - PWM: pulse width (e.g. 1000–2000 µs)
 *   - DShot: digital value (48–2047)
 *
 * @note
 * - writeThrottle() must be safe to call at runtime frequency.
 * - Implementations may have timing constraints.
 *
 * @warning
 * No safety checks (arming, limits, ramping) are enforced at this level.
 * These must be handled by higher-level control logic.
 */
class EscSignalDriver
{
public:
    virtual ~EscSignalDriver() = default;

    // Lifecycle
    virtual bool init() = 0;   // allocate/setup hardware (PWM, RMT, etc.), return true on success
    virtual void deinit() = 0; // free ALL hardware resources

    /** Write a throttle value in percent [0–100] */
    virtual void writeThrottle(float percent) = 0;

    // Safety / control helpers
    virtual void writeIdle() = 0; // stop throttle but keep ESC armed

    virtual ESCConfig getConfig() const { return _config; }
    virtual bool setConfig(ESCConfig &config);

    virtual void update() {};

protected:
    bool _initialized = false;
    ESCConfig _config;
};

#endif // ESC_SIGNAL_DRIVER_H
