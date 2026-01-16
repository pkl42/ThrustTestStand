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
 *
 * @section esc_calibration_states ESC Calibration State Diagram
 * (unchanged – see previous documentation)
 *
 * @section esc_testing Unit Testing and PWM Mocking
 * (unchanged – see previous documentation)
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
     * @param freq           PWM frequency in Hz.
     * @param resolution     PWM resolution in bits.
     * @param channel        ESP32 LEDC channel to use.
     */
    MotorESC(uint8_t pin,
             uint16_t freq = 50,
             uint8_t resolution = 12,
             uint8_t channel = 1);

    /**
     * @brief set the PWM Range
     *
     *
     * @param minPulseUs     Minimum PWM pulse width in microseconds.
     * @param maxPulseUs     Maximum PWM pulse width in microseconds.
     *
     */
    bool setPulseRangeUs(uint16_t minPulseUs, uint16_t maxPulseUs);

    uint16_t getMinPulseUs() const { return _minPulseUs; };

    uint16_t getMaxPulseUs() const { return _maxPulseUs; };

    /**
     * @brief Initialize the ESC hardware.
     *
     * Sets up PWM output and forces the ESC into DISARMED state
     * with zero throttle.
     */
    bool begin() override;

    /**
     * @brief Periodic update function.
     *
     * Must be called regularly to process throttle ramps
     * and maintain correct PWM output.
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
     * @brief Arm the ESC.
     *
     * Transitions from DISARMED → ARMED by holding
     * minimum throttle for the specified duration.
     *
     * @param armTimeMs Duration to hold minimum throttle (ms).
     */
    void arm(uint16_t armTimeMs = 2000);

    /**
     * @brief Set motor throttle.
     *
     * If throttle > 0% and ESC is ARMED, the state transitions
     * to RUNNING.
     *
     * @param throttlePercent Target throttle in percent [0–100].
     * @param smooth          Enable smooth acceleration ramp.
     * @param accelTimeMs     Duration of acceleration ramp (ms).
     *
     * @return The clamped throttle value actually applied.
     */
    float setThrottle(float throttlePercent,
                      bool smooth = false,
                      unsigned long accelTimeMs = 1000);

    /**
     * @brief Check if motor has reached its target throttle.
     *
     * @param tolerancePercent Allowed deviation from target.
     * @return true if within tolerance, false otherwise.
     */
    bool isAtTargetSpeed(float tolerancePercent = 2.0f) const;

    /**
     * @brief Get current applied throttle.
     */
    float getCurrentThrottle() const
    {
        return _currentThrottle;
    };

    /**
     * @brief Get the current ESC state.
     *
     * @return Current EscState.
     */
    EscState getMountState() const { return _state; }

protected:
    /**
     * @brief Immediately stop the motor at hardware level.
     *
     * May be called from ISR context.
     * Forces DISARMED state unconditionally.
     */
    void forceStopHardware() override;

private:
    void setPulseWidth(uint16_t pulseWidthUs);
    void applyThrottle(float throttlePercent);

    // ---------- Hardware configuration ----------
    uint8_t _pin;
    uint8_t _channel;
    uint16_t _minPulseUs = 1000;
    uint16_t _maxPulseUs = 2000;
    uint16_t _freq;
    uint8_t _resolution;
    uint32_t _maxDuty;

    // ---------- Runtime state ----------
    EscState _state = EscState::DISARMED; ///< Current ESC state

    float _currentThrottle = 0.0f;
    float _targetThrottle = 0.0f;
    float _startThrottle = 0.0f;

    unsigned long _stateStartTime = 0;
    unsigned long _transitionDuration = 1000;

    bool _smoothingActive = false;

    unsigned long _armStartTime = 0; ///< Arming start timestamp
    unsigned long _armDuration = 0;  ///< Required arming hold time (ms)
};

#endif // MOTOR_ESC_H
