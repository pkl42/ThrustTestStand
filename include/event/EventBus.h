/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EVENT_BUS_H
#define EVENT_BUS_H

#include <functional>
#include <vector>
#include "EventStructure.h"

/**
 * @class EventBus
 * @brief Simple publish-subscribe event dispatcher (singleton).
 *
 * Provides a lightweight mechanism to broadcast events to multiple subscribers.
 * Subscribers register callbacks which are invoked for every published event.
 *
 * @note
 * - Callbacks are executed synchronously in the context of the caller of publish().
 * - No thread-safety is guaranteed.
 * - Subscribers are never removed (lifetime = program lifetime).
 *
 * @warning
 * Be careful when publishing events from time-critical or interrupt contexts,
 * as all callbacks will be executed immediately.
 *
 * @code
 * EventBus::instance().subscribe([](const Event& e) {
 *     // handle event
 * });
 *
 * EventBus::instance().publish(myEvent);
 * @endcode
 */
class EventBus
{
public:
    using Callback = std::function<void(const Event &)>;

    /**
     * @brief Access the global EventBus instance.
     */
    static EventBus &instance()
    {
        static EventBus instance;
        return instance;
    }

    /**
     * @brief Register a new event subscriber.
     * @param cb Callback that will be invoked for every published event.
     */
    void subscribe(Callback cb)
    {
        _subscribers.push_back(cb);
    }

    /**
     * @brief Publish an event to all subscribers.
     * @param event Event to broadcast.
     */
    void publish(const Event &event)
    {
        for (auto &cb : _subscribers)
        {
            cb(event);
        }
    }

private:
    std::vector<Callback> _subscribers;
};

#endif // EVENT_BUS_H