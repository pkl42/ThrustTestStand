/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MOTOR_ESC_H
#define MOTOR_ESC_H

#include <Arduino.h>
#include <memory>
#include "BaseActuator.h"
#include "esc/DShotEscDriver.h"
#include "esc/PwmEscDriver.h"

/**
 * @class MotorESC
 * @brief High-level ESC motor controller with safety, state handling, and smoothing.
 *
 * Wraps a protocol-specific @ref EscSignalDriver and provides:
 * - Arming / disarming logic
 * - Throttle state machine
 * - Optional throttle smoothing (ramping)
 * - Emergency stop handling
 *
 * @section esc_arch Architecture
 * MotorESC acts as a safety and control layer above the raw ESC driver:
 *
 * MotorESC (state + safety)
 *        ↓
 * EscSignalDriver (PWM / DShot)
 *
 * @section esc_thread_safety Thread / Execution Context Safety
 * - All public methods must be called from non-ISR context.
 * - ISR-safe only: forceStopHardware()
 *
 * @section esc_state_model ESC Internal State Model
 * (existing diagram unchanged)
 *
 * @section esc_behavior Key Behavior
 * - Throttle commands are ignored unless ESC is ARMED or RUNNING
 * - Smoothing is time-based and handled in update()
 * - Driver always receives normalized throttle [0–100%]
 *
 * @warning
 * This class does NOT validate physical motor response (no RPM feedback).
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
    enum class MountState : uint8_t
    {
        DISARMED = 0, ///< Safe state, throttle forced to zero
        ARMING = 1,   ///< Waiting for arm hold time to elapse
        ARMED = 2,    ///< ESC armed, zero throttle, ready
        RUNNING = 3   ///< Throttle > 0%, motor spinning
    };

    /**
     * @brief Construct a MotorESC instance
     */
    explicit MotorESC(uint8_t escPin);

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
     * Starts arming sequence by holding minimum throttle for a defined duration.
     *
     * @param armTimeMs Duration to hold minimum throttle (ms)
     * @return true if arming process started
     *
     * @note
     * - Actual transition to ARMED occurs after armTimeMs in update().
     * - While ARMING, throttle commands are ignored.
     */
    bool arm(uint16_t armTimeMs = 2000);

    /**
     * @brief Disarm the ESC.
     *
     * Forces throttle to zero and transitions to DISARMED.
     *
     * @param forceFlag If true, bypasses state restrictions
     * @return true if disarm was successful
     *
     * @note
     * Should be called before changing driver or configuration.
     */
    bool disarm(bool forceFlag = false);

    /**
     * @brief Set motor throttle.
     *
     * If throttle > 0% and ESC is ARMED, transitions to RUNNING.
     *
     * @param throttlePercent Target throttle in percent [0–100]
     * @param smooth Enable time-based smoothing (ramp)
     * @param transitionTimeMs Duration of ramp (ms)
     *
     * @return The clamped throttle value actually applied
     *
     * @note
     * - If smoothing is enabled, the target is reached gradually
     *   via update().
     * - If smoothing is disabled, throttle is applied immediately.
     *
     * @warning
     * Calling this while DISARMED has no effect.
     */
    float setThrottle(float throttlePercent,
                      bool smooth = false,
                      unsigned long transitionTimeMs = 1000);

    /**
     * @brief Check if motor has reached its target throttle.
     *
     * @param tolerancePercent Allowed deviation from target
     * @return true if current throttle is within tolerance
     *
     * @note
     * Useful when using smoothing to detect ramp completion.
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
     * @return Current MountState.
     */
    MountState getMountState() const { return _mountState; }

    ESCConfig getDriverConfig() const { return _driverConfig; }

    /**
     * @brief Set driver-specific configuration.
     *
     * Applies configuration parameters to the underlying driver.
     *
     * @param driverConfig Configuration to apply
     * @return true if successfully applied
     *
     * @note
     * - Not all parameters are used by all drivers.
     * - Should only be called while DISARMED.
     */
    bool setDriverConfig(ESCConfig &driverConfig);

    /**
     * @brief Set the ESC driver type.
     *
     * Creates and initializes the corresponding driver implementation
     * (e.g. PWM or DShot).
     *
     * @param type Desired driver type
     * @return true if driver was created and initialized successfully
     *
     * @note
     * - Replaces any previously active driver.
     * - Should only be called while DISARMED.
     */
    bool setDriver(EscDriverType type);

    /**
     * @brief Get currently active driver type.
     */
    EscDriverType getDriverType() const { return _escDriverType; };

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
     * @brief Update the driver with the current throttle value.
     *
     * Converts logical throttle into driver output.
     */
    void updateThrottle(float throttlePercent);

    std::unique_ptr<EscSignalDriver> _driver; ///< owned ESC driver
    ESCConfig _driverConfig;

    EscDriverType _escDriverType = EscDriverType::ESC_DRIVER_NONE;
    uint8_t _escPin;

    MountState _mountState = MountState::DISARMED; ///< Current ESC state

    /**
     * @brief Reset all throttle-related state variables.
     *
     * Used during stop/disarm transitions.
     */
    void resetThrottleValues();

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
