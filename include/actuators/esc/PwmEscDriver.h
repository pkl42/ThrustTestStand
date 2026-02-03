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
 * Handles pulse-width modulation to control standard RC ESCs.
 * Supports configurable pulse range, frequency, and resolution.
 */
class PwmEscDriver : public EscSignalDriver
{
public:
    /**
     * @brief Construct a PWM ESC driver
     * @param pin        GPIO connected to ESC signal
     * @param freq       PWM frequency (Hz), typically 50
     * @param resolution PWM resolution in bits, max 16
     * @param channel    LEDC channel
     */
    PwmEscDriver(uint8_t pin, uint16_t freq = 50, uint8_t resolution = 12, uint8_t channel = 1);

    bool begin() override;
    void writeThrottle(float percent) override;
    void writeStop() override;

    /**
     * @brief Set pulse width range for 0% to 100% throttle
     * @param minUs minimum pulse width (µs)
     * @param maxUs maximum pulse width (µs)
     */
    bool setPulseRangeUs(uint16_t minUs, uint16_t maxUs) override;

    /**
     * @brief Get configured minimum PWM pulse width.
     *
     * @return Minimum pulse width in microseconds.
     */
    uint16_t getMinPulseUs() const override { return _minPulseUs; }
    /**
     * @brief Get configured maximum PWM pulse width.
     *
     * @return Maximum pulse width in microseconds.
     */
    uint16_t getMaxPulseUs() const override { return _maxPulseUs; }

    void stop() override;

private:
    void setPulseWidth(uint16_t pulseUs);

    uint8_t _pin;
    uint8_t _channel;
    uint16_t _freq;
    uint8_t _resolution;
    uint16_t _maxDuty;

    uint16_t _minPulseUs = 1000;
    uint16_t _maxPulseUs = 2000;
};

#endif // PWM_ESC_DRIVER_H
