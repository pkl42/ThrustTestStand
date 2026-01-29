/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MOTOR_ESC_H
#define MOTOR_ESC_H

#include <Arduino.h>
#include "BaseActuator.h"

/**
 * @class MotorESC
 * @brief Electronic Speed Controller (ESC) motor actuator for ESP32.
 *
 * This class implements a PWM-based motor controller suitable for
 * RC-style Electronic Speed Controllers (ESCs). It uses the ESP32 LEDC
 * peripheral to generate servo-style PWM signals (typically 50 Hz).
 *
 * The class supports:
 * - Explicit ESC arming and calibration sequences
 * - Throttle limiting
 * - Optional smooth acceleration ramps
 * - Emergency stop integration via BaseActuator
 *
 * @section esc_thread_safety Thread / Execution Context Safety
 * - All public methods must be called from non-ISR context.
 * - This class is **not ISR-safe** except for `forceStopHardware()`,
 *   which may be invoked from an emergency stop ISR.
 *
 * @section esc_state_model ESC Internal State Model
 *
 * The ESC operates as a simple explicit state machine:
 *
 * @code
 * ┌──────────┐
 * │ DISARMED │  ← default after begin()
 * └────┬─────┘
 *      │ arm()
 *      ▼
 * ┌──────────┐
 * │  ARMED   │  ← ready to accept throttle
 * └────┬─────┘
 *      │ setThrottle() > 0
 *      ▼
 * ┌──────────┐
 * │ RUNNING  │
 * └────┬─────┘
 *      │ stop() / forceStopHardware()
 *      ▼
 * ┌──────────┐
 * │ DISARMED │
 * └──────────┘
 * @endcode
 *
 * State invariants:
 * - Throttle output is **always 0%** in DISARMED
 * - Throttle > 0% is only allowed in RUNNING
 * - Calling `setThrottle()` while DISARMED does not start the motor
 * - Emergency stop forces DISARMED immediately
 */
class MotorESC : public BaseActuator
{
public:
    /**
     * @brief ESC operating states.
     *
     * These states reflect the logical control state of the ESC,
     * independent of the physical motor speed.
     */
    enum class EscState : uint8_t
    {
        DISARMED = 0, ///< Safe state, throttle forced to zero
        ARMING,       ///< Waiting for arm hold time to elapse
        ARMED,        ///< ESC armed, zero throttle, ready
        RUNNING       ///< Throttle > 0%, motor spinning
    };

    /**
     * @brief Construct a MotorESC instance.
     *
     * @param pin            GPIO pin connected to the ESC signal wire.
     * @param freq           PWM frequency in Hz (typically 50 Hz).
     * @param resolution     PWM resolution in bits.
     * @param channel        ESP32 LEDC channel to use.
     */
    MotorESC(uint8_t pin,
             uint16_t freq = 50,
             uint8_t resolution = 12,
             uint8_t channel = 1);

    /**
     * @brief Configure the ESC PWM pulse width range.
     *
     * Defines the minimum and maximum pulse widths corresponding
     * to 0% and 100% throttle.
     *
     * @param minPulseUs     Minimum PWM pulse width in microseconds.
     * @param maxPulseUs     Maximum PWM pulse width in microseconds.
     *
     * @return true if the range is valid and applied, false otherwise.
     */
    bool setPulseRangeUs(uint16_t minPulseUs, uint16_t maxPulseUs);

    /**
     * @brief Get configured minimum PWM pulse width.
     *
     * @return Minimum pulse width in microseconds.
     */
    uint16_t getMinPulseUs() const { return _minPulseUs; }

    /**
     * @brief Get configured maximum PWM pulse width.
     *
     * @return Maximum pulse width in microseconds.
     */
    uint16_t getMaxPulseUs() const { return _maxPulseUs; }

    /**
     * @brief Initialize the ESC hardware.
     *
     * Sets up PWM output and forces the ESC into DISARMED state
     * with zero throttle.
     *
     * @return true on successful initialization, false on failure.
     */
    bool begin() override;

    /**
     * @brief Periodic update function.
     *
     * Must be called regularly to process throttle ramps
     * and maintain correct PWM output.
     *
     * @return true if update succeeded, false otherwise.
     */
    bool update() override;

    /**
     * @brief Stop the motor and disarm the ESC.
     *
     * Transitions the ESC to DISARMED state and forces
     * throttle output to zero.
     */
    void stop() override;

    /**
     * @brief Stop the motor due to an error condition.
     *
     * Behaves like stop(), but is intended to be invoked
     * when a fault or safety violation is detected.
     */
    void stopWithError() override;

    /**
     * @brief Arm the ESC.
     *
     * Transitions from DISARMED → ARMED by holding
     * minimum throttle for the specified duration.
     *
     * @param armTimeMs Duration to hold minimum throttle (ms).
     * @return true if arming sequence started successfully.
     */
    bool arm(uint16_t armTimeMs = 2000);

    /**
     * @brief Disarm the ESC without requiring an error or stop event.
     *
     * Forces throttle to zero and transitions to DISARMED.
     *
     * @return true if disarm was successful.
     */
    bool disarm();

    /**
     * @brief Set motor throttle.
     *
     * If throttle > 0% and ESC is ARMED, the state transitions
     * to RUNNING.
     *
     * @param throttlePercent Target throttle in percent [0–100].
     * @param smooth          Enable smooth acceleration ramp.
     * @param transitionTimeMs Duration of acceleration ramp (ms).
     *
     * @return The clamped throttle value actually applied.
     */
    float setThrottle(float throttlePercent,
                      bool smooth = false,
                      unsigned long transitionTimeMs = 1000);

    /**
     * @brief Check if motor has reached its target throttle.
     *
     * @param tolerancePercent Allowed deviation from target.
     * @return true if within tolerance, false otherwise.
     */
    bool isAtTargetSpeed(float tolerancePercent = 2.0f) const;

    /**
     * @brief Get the currently applied throttle value.
     *
     * @return Current throttle in percent.
     */
    float getCurrentThrottle() const { return _currentThrottle; }

    /**
     * @brief Get the current ESC control state.
     *
     * @return Current EscState.
     */
    EscState getMountState() const { return _escState; }

protected:
    /**
     * @brief Immediately stop the motor at hardware level.
     *
     * May be called from ISR context.
     * Forces DISARMED state unconditionally.
     */
    void forceStopHardware() override;

private:
    /**
     * @brief Set raw PWM pulse width.
     *
     * @param pulseWidthUs Pulse width in microseconds.
     */
    void setPulseWidth(uint16_t pulseWidthUs);

    /**
     * @brief Apply throttle percentage to PWM output.
     *
     * @param throttlePercent Throttle in percent [0–100].
     */
    void applyThrottle(float throttlePercent);

    // ---------- Hardware configuration ----------
    uint8_t _pin;                ///< GPIO pin connected to ESC signal
    uint8_t _channel;            ///< LEDC PWM channel
    uint16_t _minPulseUs = 1000; ///< Minimum PWM pulse width (µs)
    uint16_t _maxPulseUs = 2000; ///< Maximum PWM pulse width (µs)
    uint16_t _freq;              ///< PWM frequency (Hz)
    uint8_t _resolution;         ///< PWM resolution (bits)
    uint32_t _maxDuty;           ///< Maximum LEDC duty cycle value

    // ---------- Runtime state ----------
    EscState _escState = EscState::DISARMED; ///< Current ESC state

    float _currentThrottle = 0.0f; ///< Currently applied throttle (%)
    float _targetThrottle = 0.0f;  ///< Desired target throttle (%)
    float _startThrottle = 0.0f;   ///< Throttle at start of transition (%)

    unsigned long _stateStartTime = 0;      ///< Timestamp of last state change
    unsigned long _transitionTimeMs = 1000; ///< Throttle transition duration

    bool _smoothingActive = false; ///< Throttle ramp active flag

    unsigned long _armStartTime = 0; ///< Arming start timestamp
    unsigned long _armDuration = 0;  ///< Required arming hold time (ms)
};

#endif // MOTOR_ESC_H
