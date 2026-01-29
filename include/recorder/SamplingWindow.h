/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SAMPLING_WINDOW_H
#define SAMPLING_WINDOW_H

#include <Arduino.h>

/**
 * @class SamplingWindow
 * @brief Time-based sampling window helper.
 *
 * Provides a lightweight utility for managing fixed-duration
 * sampling windows using the Arduino millis() time base.
 *
 * The class is intentionally minimal and non-blocking, and is
 * typically used to delimit time intervals during which sensor
 * data is accumulated.
 *
 * @section sampling_window_timing Timing Semantics
 * - Time is measured using millis()
 * - Window duration is inclusive of the start time
 * - Expiration is checked lazily via expired()
 *
 * @section sampling_window_thread_safety Thread / Execution Context Safety
 * - All methods must be called from non-ISR context
 * - This class is not ISR-safe
 */
class SamplingWindow
{
public:
    /**
     * @brief Start a new sampling window.
     *
     * Activates the window and records the current time
     * as the window start.
     *
     * If a window is already active, it is restarted.
     *
     * @param windowMs Duration of the sampling window in milliseconds.
     */
    void begin(uint32_t windowMs)
    {
        _windowMs = windowMs;
        _active = true;
        _startMs = millis();
    }

    /**
     * @brief Check whether the sampling window is active.
     *
     * @return true if a window is currently active.
     */
    bool isActive() const { return _active; }

    /**
     * @brief Check whether the sampling window has expired.
     *
     * A window is considered expired when the configured
     * duration has elapsed since begin() was called.
     *
     * @return true if the window is active and expired,
     *         false otherwise.
     */
    bool expired() const
    {
        return _active && (millis() - _startMs >= _windowMs);
    }

    /**
     * @brief Stop the sampling window.
     *
     * Deactivates the window without modifying timing data.
     */
    void stop()
    {
        _active = false;
    }

private:
    bool _active = false;   ///< Window active flag
    uint32_t _windowMs = 0; ///< Window duration (ms)
    uint32_t _startMs = 0;  ///< Window start timestamp (ms)
};

#endif // SAMPLING_WINDOW_H
