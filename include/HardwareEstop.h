#ifndef HARDWARE_ESTOP_H
#define HARDWARE_ESTOP_H

#include <Arduino.h>
#include <vector>
#include "BaseActuator.h"

/**
 * @class HardwareEstop
 * @brief Hardware-based emergency stop handler.
 *
 * This class monitors a dedicated GPIO pin connected to a physical
 * emergency stop (E-Stop) switch. When the E-Stop is triggered, all
 * attached actuators are immediately commanded to stop.
 *
 * The E-Stop is handled via a GPIO interrupt for minimal latency.
 * Once triggered, the state remains latched until a system reset
 * or reinitialization.
 *
 * @section estop_thread_safety Thread / ISR Safety
 * - The GPIO interrupt handler runs in ISR context.
 * - The internal `_triggered` flag is declared `volatile` and may be
 *   accessed from both ISR and non-ISR code.
 * - `handleInterrupt()` must remain ISR-safe:
 *   - No dynamic memory allocation
 *   - No blocking calls
 *   - No logging or serial output
 * - All attached actuators **must** implement an emergency stop method
 *   that is safe to call from an ISR context (or internally defer
 *   heavy processing).
 *
 * @section estop_usage_example Usage Example
 * @code
 * HardwareEstop estop(25); // GPIO 25, active LOW E-Stop
 *
 * MotorController motor;
 * ValveController valve;
 *
 * void setup()
 * {
 *     motor.begin();
 *     valve.begin();
 *
 *     estop.attachActuator(&motor);
 *     estop.attachActuator(&valve);
 *     estop.begin();
 * }
 *
 * void loop()
 * {
 *     if (estop.isTriggered())
 *     {
 *         // Optional: enter safe state, disable UI, signal fault, etc.
 *         while (true)
 *         {
 *             delay(100);
 *         }
 *     }
 * }
 * @endcode
 */
class HardwareEstop
{
public:
    /**
     * @brief Construct a HardwareEstop instance.
     *
     * @param gpioPin   GPIO pin connected to the E-Stop switch.
     * @param activeLow Set to true if the E-Stop switch pulls the pin LOW
     *                  when triggered (default: true).
     */
    explicit HardwareEstop(uint8_t gpioPin, bool activeLow = true);

    /**
     * @brief Initialize the E-Stop hardware.
     *
     * Configures the GPIO pin and attaches the interrupt handler.
     * Must be called once during system startup.
     */
    void begin();

    /**
     * @brief Attach an actuator to be stopped on E-Stop.
     *
     * All attached actuators will have their emergency stop
     * mechanism invoked when the E-Stop is triggered.
     *
     * @note This method must be called before begin().
     *
     * @param actuator Pointer to a BaseActuator instance.
     */
    void attachActuator(BaseActuator *actuator);

    /**
     * @brief Check whether the E-Stop has been triggered.
     *
     * This method is safe to call from the main loop or tasks.
     *
     * @return true if the E-Stop has been activated, false otherwise.
     */
    bool isTriggered() const;

private:
    /**
     * @brief Static ISR wrapper for GPIO interrupts.
     *
     * This function is placed in IRAM and forwards the interrupt
     * to the corresponding HardwareEstop instance.
     *
     * @param arg Pointer to the HardwareEstop instance.
     */
    static void IRAM_ATTR isrHandler(void *arg);

    /**
     * @brief Instance-level interrupt handler.
     *
     * Latches the triggered state and commands all attached
     * actuators to stop immediately.
     *
     * @warning Runs in ISR context. Must remain ISR-safe.
     */
    void handleInterrupt();

    uint8_t _pin;    ///< GPIO pin used for the E-Stop input
    bool _activeLow; ///< True if the E-Stop signal is active LOW

    volatile bool _triggered = false; ///< Latched E-Stop state set by ISR

    std::vector<BaseActuator *> _actuators; ///< Actuators affected by E-Stop
};

#endif
