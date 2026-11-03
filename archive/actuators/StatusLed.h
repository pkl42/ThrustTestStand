/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>

/**
 * @class StatusLed
 * @brief System status LED controller.
 *
 * Provides a simple non-blocking state-driven interface for controlling
 * a multi-color status LED. The LED reflects the overall system state
 * using color and blinking patterns.
 *
 * Blinking behavior is implemented using time-based toggling and
 * therefore requires periodic calls to update().
 *
 * @section status_led_thread_safety Thread / Execution Context Safety
 * - All public methods must be called from non-ISR context.
 * - This class is not ISR-safe.
 *
 * @section status_led_states State Semantics
 *
 * Each logical state maps to a predefined LED color and blink pattern:
 *
 * - OFF:     LED disabled
 * - READY:   Green, steady on
 * - BUSY:    Yellow, steady or slow blink
 * - RUNNING: Green, blinking
 * - WARNING: Yellow, blinking
 * - ERROR:   Red, blinking
 *
 * The exact blink timing and color mapping are implementation-defined.
 */
class StatusLed
{
public:
    /**
     * @brief Logical LED states.
     *
     * These states represent high-level system conditions
     * and are translated internally into colors and blink patterns.
     */
    enum class State
    {
        OFF,     ///< LED off
        READY,   ///< Green steady
        BUSY,    ///< Yellow steady or slow blink
        RUNNING, ///< Green blinking
        WARNING, ///< Yellow blinking
        ERROR    ///< Red blinking
    };

    /**
     * @brief Initialize the status LED hardware.
     *
     * Configures GPIOs and sets the LED to the OFF state.
     * Must be called before any other method.
     */
    void begin();

    /**
     * @brief Set the current LED state.
     *
     * Updates the logical LED state. Visual changes may not
     * occur immediately if the state uses blinking and will
     * be handled in update().
     *
     * @param state New LED state.
     */
    void setState(State state);

    /**
     * @brief Periodic update function.
     *
     * Must be called regularly to handle blinking behavior
     * and time-based LED transitions.
     */
    void update();

private:
    State _state = State::OFF; ///< Current logical LED state
    uint32_t _lastToggle = 0;  ///< Timestamp of last blink toggle
    bool _ledOn = true;        ///< Current LED on/off phase

    /**
     * @brief Apply the current LED color and on/off state.
     *
     * @param on true to turn LED on, false to turn it off.
     */
    void applyColor(bool on);
};

#endif // STATUS_LED_H
