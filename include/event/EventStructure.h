/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EVENT_STRUCTURE_H
#define EVENT_STRUCTURE_H

#include <stdint.h>

/**
 * @file EventStructure.h
 * @brief Basic event definitions used by the EventBus system.
 */

/**
 * @enum EventType
 * @brief High-level categories of events.
 *
 * Used to classify events before evaluating the more specific @ref Event::code.
 */
enum class EventType
{
    SYSTEM_STATE_CHANGE, ///< Changes in system state (e.g. arming, errors)
    NETWORK_STATE_CHANGE ///< Network connectivity or status changes
    // SENSOR,
    // CALIBRATION,
};

/**
 * @struct Event
 * @brief Generic event container passed through the EventBus.
 *
 * An event consists of a high-level @ref EventType and a more specific
 * application-defined code.
 *
 * @note
 * The meaning of @ref code depends on the associated @ref EventType.
 * It is recommended to define separate enums per EventType for clarity.
 *
 * @code
 * Event e(EventType::SYSTEM_STATE_CHANGE, MOTOR_DISARMED, millis());
 * EventBus::instance().publish(e);
 * @endcode
 */
struct Event
{
    EventType type;     ///< Event category
    uint16_t code = 0;  ///< Type-specific event code (user-defined)
    uint32_t timestamp; ///< Timestamp of event creation (e.g. ms since boot)

    /**
     * @brief Construct a new Event.
     * @param t Event type
     * @param c Event-specific code
     * @param ts Timestamp (typically system time in ms)
     */
    Event(EventType t, uint16_t c, uint32_t ts)
        : type(t), code(c), timestamp(ts) {}
};

#endif // EVENT_STRUCTURE_H