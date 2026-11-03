/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ESC_SIGNAL_DRIVER_H
#define ESC_SIGNAL_DRIVER_H

/**
 * @brief ESC Driver / Signal Protocol Type.
 *
 * Defines which ESC driver the thrust stand uses for motor control.
 * This also allows reporting the driver used in test logs and CSVs.
 */
enum class EscDriverType : uint8_t
{
    ESC_DRIVER_NONE = 0, ///< No driver selected / uninitialized
    ESC_DRIVER_PWM = 1,  ///< Standard servo-style PWM
    ESC_DRIVER_DSHOT150 = 2,
    ESC_DRIVER_DSHOT300 = 3,
    ESC_DRIVER_DSHOT600 = 4
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
    case EscDriverType::ESC_DRIVER_DSHOT150:
        return "D-Shot 150";
    case EscDriverType::ESC_DRIVER_DSHOT300:
        return "D-Shot 300";
    case EscDriverType::ESC_DRIVER_DSHOT600:
        return "D-Shot 600";
    default:
        return "Unknown ESC Driver";
    }
}

/**
 * @brief Abstract interface for ESC signal generation.
 *
 * Implementations may use PWM, D-Shot, or any other protocol.
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

    virtual uint16_t getMinPulseUs() const { return 1000; }  // default dummy
    virtual uint16_t getMaxPulseUs() const { return 2000; }  // default dummy
    virtual uint16_t getCurrentPulseUs() const { return 0; } // default dummy

    virtual bool setPulseRangeUs(uint16_t minUs, uint16_t maxUs) { return false; }

    virtual void update() {};

protected:
    bool _initialized = false;
};

#endif // ESC_SIGNAL_DRIVER_H
