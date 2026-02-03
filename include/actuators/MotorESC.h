/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MOTOR_ESC_H
#define MOTOR_ESC_H

#include <Arduino.h>
#include "BaseActuator.h"
#include "esc/EscSignalDriver.h"

/**
 * @class MotorESC
 * @brief Electronic Speed Controller (ESC) motor actuator for ESP32.
 *
 * This class implements a motor controller with arming, smoothing,
 * throttle limiting, and emergency stop support. The actual ESC
 * signal output is delegated to an EscSignalDriver (PWM, D-Shot, etc.).
 *
 * @section esc_thread_safety Thread / Execution Context Safety
 * - All public methods must be called from non-ISR context.
 * - ISR-safe only: forceStopHardware()
 *
 * @section esc_state_model ESC Internal State Model
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
 * - Throttle output is 0% in DISARMED
 * - Throttle > 0% only in RUNNING
 * - Calling setThrottle() while DISARMED does nothing
 * - Emergency stop forces DISARMED immediately
 */
class MotorESC : public BaseActuator
{
public:
    /**
     * @brief ESC operating states.
     *
     * Reflects the logical control state of the ESC,
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
     * @brief Construct a MotorESC instance with a signal driver.
     *
     * @param driver Pointer to an EscSignalDriver implementation (PWM, D-Shot, etc.)
     */
    explicit MotorESC(EscSignalDriver *driver);

    /**
     * @brief Initialize the ESC hardware via the driver.
     *
     * Forces the ESC into DISARMED state with zero throttle.
     *
     * @return true on success, false on failure.
     */
    bool begin() override;

    /**
     * @brief Periodic update function.
     *
     * Must be called regularly to process throttle ramps
     * and maintain correct output.
     *
     * @return true if update succeeded, false otherwise.
     */
    bool update() override;

    void updateEsc();

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
     * @brief Disarm the ESC.
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

    /**
     * @brief Get configured minimum PWM pulse width.
     *
     * @return Minimum pulse width in microseconds.
     */
    uint16_t getMinPulseUs() const { return _driver ? _driver->getMinPulseUs() : 1000; }
    /**
     * @brief Get configured maximum PWM pulse width.
     *
     * @return Maximum pulse width in microseconds.
     */
    uint16_t getMaxPulseUs() const { return _driver ? _driver->getMaxPulseUs() : 2000; }
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

    bool setDriver(EscSignalDriver *driver);

    void writeStop();

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
     * @brief Update the driver with the target throttle.
     *
     * @param throttlePercent Throttle in percent [0–100]
     */
    void updateThrottle(float throttlePercent);

    EscSignalDriver *_driver; ///< Pointer to signal driver

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
