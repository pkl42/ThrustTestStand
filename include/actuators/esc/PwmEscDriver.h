/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PWM_ESC_DRIVER_H
#define PWM_ESC_DRIVER_H

#include "Arduino.h"
#include "actuators/esc/EscSignalDriver.h"

/**
 * @class PwmEscDriver
 * @brief PWM-based ESC driver using ESP32 LEDC peripheral.
 *
 * Generates a servo-style PWM signal to control standard RC ESCs.
 * The throttle is encoded as pulse width (typically 1000–2000 µs).
 *
 * @note
 * - Uses ESP32 LEDC hardware (timer + channel).
 * - Typical ESCs expect ~50 Hz update rate.
 * - Pulse width range is defined via @ref setConfig().
 *
 * @warning
 * - Incorrect pulse ranges may prevent arming or cause unsafe behavior.
 * - No automatic ESC calibration is performed.
 *
 * @see DShotEscDriver
 */
class PwmEscDriver : public EscSignalDriver
{
public:
    /**
     * @brief Construct a PWM ESC driver.
     * @param pin        GPIO connected to ESC signal
     * @param freq       PWM frequency in Hz (typically 50)
     * @param resolution PWM resolution in bits (max 16)
     * @param channel    LEDC channel index
     */
    PwmEscDriver(uint8_t pin, uint16_t freq = 50, uint8_t resolution = 12, uint8_t channel = 1);

    /**
     * @brief Initialize LEDC timer and channel.
     */
    bool init() override;

    /**
     * @brief Set throttle as percentage.
     * @param percent Throttle in range [0.0, 100.0]
     *
     * Converted internally to a pulse width between configured
     * minimum and maximum values (e.g. 1000–2000 µs).
     */
    void writeThrottle(float percent) override;

    /**
     * @brief Send idle (minimum throttle) signal.
     */
    void writeIdle() override;

    /**
     * @brief Apply ESC configuration (pulse range, limits, etc.).
     */
    bool setConfig(ESCConfig &config) override;

    /**
     * @brief Release LEDC resources.
     */
    void deinit() override;

private:
    /**
     * @brief Set raw pulse width.
     * @param pulseUs Pulse width in microseconds
     */
    void setPulseWidth(uint16_t pulseUs);

    uint8_t _pin;        ///< Output GPIO
    uint8_t _channel;    ///< LEDC channel
    uint16_t _freq;      ///< PWM frequency (Hz)
    uint8_t _resolution; ///< PWM resolution (bits)
    uint16_t _maxDuty;   ///< Maximum duty value (derived from resolution)

    uint16_t _pulseUs = 0; ///< Last applied pulse width (µs)
};

#endif // PWM_ESC_DRIVER_H
