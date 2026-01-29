/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <Arduino.h>
#include "actuators/StatusLed.h"
#include "core/Config.h"

/* =========================================================
   REAL IMPLEMENTATION (board has onboard RGB LED)
   ========================================================= */
#if defined(HAS_RGB_LED)

#ifndef RGB_BRIGHTNESS
#define RGB_BRIGHTNESS 32
#endif

static StatusLed::State _state = StatusLed::State::OFF;
static bool _ledOn = true;
static uint32_t _lastToggle = 0;

void StatusLed::begin()
{
    setState(State::BUSY);
}

void StatusLed::setState(State state)
{
    if (state == _state)
        return;
    _state = state;
    _lastToggle = millis();
    _ledOn = true;
}

void StatusLed::update()
{
    uint32_t now = millis();

    switch (_state)
    {
    case State::RUNNING:
    case State::WARNING:
    case State::ERROR:
        if (now - _lastToggle > 500)
        {
            _ledOn = !_ledOn;
            _lastToggle = now;
        }
        break;

    default:
        _ledOn = true;
        break;
    }

    if (!_ledOn)
    {
        neopixelWrite(RGB_BUILTIN_LED, 0, 0, 0);
        return;
    }

    switch (_state)
    {
    case State::READY:
    case State::RUNNING:
        neopixelWrite(RGB_BUILTIN_LED, 0, RGB_BRIGHTNESS, 0);
        break;

    case State::BUSY:
    case State::WARNING:
        neopixelWrite(RGB_BUILTIN_LED, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0);
        break;

    case State::ERROR:
        neopixelWrite(RGB_BUILTIN_LED, RGB_BRIGHTNESS, 0, 0);
        break;

    default:
        neopixelWrite(RGB_BUILTIN_LED, 0, 0, 0);
        break;
    }
}

#else
/* =========================================================
   FALLBACK STUB (no RGB LED on board)
   ========================================================= */

void StatusLed::begin() {}
void StatusLed::setState(State) {}
void StatusLed::update() {}

#endif
