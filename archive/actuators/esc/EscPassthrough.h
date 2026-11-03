/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PASS_THROUGH_H
#define PASS_THROUGH_H

#include <Arduino.h>

class EscPassthrough
{
public:
    static bool begin(uint8_t pin);
    static void end();

    // returns true if data moved
    static bool update();

    static bool isActive();

private:
    static bool _active;
    static uint8_t _esc_pin;

    // --- Pin mode helpers ---
    static void escPinTx();
    static void escPinRx();

    // --- ESC protocol ---
    static void sendByte(uint8_t b);

    // --- RX handling ---
    static void IRAM_ATTR gpio_isr(void *arg);
    static void processEdge(uint32_t now);

    // --- ring buffer ---
    static constexpr size_t RX_BUF_SIZE = 128;
    static volatile uint8_t _rxBuf[RX_BUF_SIZE];
    static volatile uint16_t _rxHead;
    static volatile uint16_t _rxTail;

    static bool rxAvailable();
    static uint8_t rxRead();

    // --- bit decode ---
    static volatile uint8_t _rxBitIndex;
    static volatile uint8_t _rxByte;
    static volatile uint32_t _lastEdge;

    // --- TX burst state ---
    static volatile bool _txActive;
};

#endif