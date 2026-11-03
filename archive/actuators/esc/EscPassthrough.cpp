/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "actuators/esc/EscPassthrough.h"

#include <driver/rmt.h>
#include <driver/gpio.h>
#include <esp_log.h>

static const char *TAG = "EscPassthrough";

/*
BLHeli/AM32 passthrough timing

1 → 18µs HIGH + 9µs LOW
0 → 9µs HIGH + 18µs LOW
*/

static constexpr uint16_t T1H = 18;
static constexpr uint16_t T1L = 9;
static constexpr uint16_t T0H = 9;
static constexpr uint16_t T0L = 18;

bool EscPassthrough::_active = false;
uint8_t EscPassthrough::_esc_pin = 255;

static bool _gpioIsrInstalled = false;

volatile uint8_t EscPassthrough::_rxBuf[RX_BUF_SIZE];
volatile uint16_t EscPassthrough::_rxHead = 0;
volatile uint16_t EscPassthrough::_rxTail = 0;

volatile uint8_t EscPassthrough::_rxBitIndex = 0;
volatile uint8_t EscPassthrough::_rxByte = 0;
volatile uint32_t EscPassthrough::_lastEdge = 0;

volatile bool EscPassthrough::_txActive = false;

static rmt_channel_t RMT_TX_CHANNEL = RMT_CHANNEL_0;

static rmt_item32_t tx_items[8];

// ------------------------------------------------
// Pin control
// ------------------------------------------------

void EscPassthrough::escPinTx()
{
    gpio_set_direction((gpio_num_t)_esc_pin, GPIO_MODE_OUTPUT);
}

void EscPassthrough::escPinRx()
{
    gpio_set_direction((gpio_num_t)_esc_pin, GPIO_MODE_INPUT);
}

// ------------------------------------------------
// RMT init
// ------------------------------------------------

static void rmt_init(uint8_t pin)
{
    rmt_config_t config = {};

    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_TX_CHANNEL;
    config.gpio_num = (gpio_num_t)pin;
    config.mem_block_num = 1;

    config.clk_div = 80; // 1 tick = 1 µs (80MHz / 80)

    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    config.tx_config.idle_output_en = true;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

// ------------------------------------------------
// Begin / End
// ------------------------------------------------

bool EscPassthrough::begin(uint8_t pin)
{
    _esc_pin = pin;
    _active = true;

    // Initialize RMT peripheral for this pin
    rmt_init(pin);

    // Set pin to input for receiving ESC data
    escPinRx();

    // Configure GPIO interrupt for any edge
    gpio_set_intr_type((gpio_num_t)_esc_pin, GPIO_INTR_ANYEDGE);

    // Install ISR service only once (ESP32 global)
    static bool isr_installed = false;
    if (!isr_installed)
    {
        gpio_install_isr_service(0);
        isr_installed = true;
    }

    // Attach ISR handler for this ESC pin
    gpio_isr_handler_add((gpio_num_t)_esc_pin, gpio_isr, nullptr);

    ESP_LOGI(TAG, "ESC passthrough started on pin %d", pin);

    return true;
}

void EscPassthrough::end()
{
    if (!_active)
        return;

    _active = false;

    // Remove ISR handler for this ESC pin
    gpio_isr_handler_remove((gpio_num_t)_esc_pin);

    // Set pin back to safe input state
    escPinRx();

    ESP_LOGI(TAG, "ESC passthrough stopped on pin %d", _esc_pin);
}

// ------------------------------------------------
// TX
// ------------------------------------------------

void EscPassthrough::sendByte(uint8_t b)
{
    escPinTx();

    for (int i = 0; i < 8; i++)
    {
        bool bit = (b >> i) & 1;

        if (bit)
        {
            tx_items[i].level0 = 1;
            tx_items[i].duration0 = T1H;
            tx_items[i].level1 = 0;
            tx_items[i].duration1 = T1L;
        }
        else
        {
            tx_items[i].level0 = 1;
            tx_items[i].duration0 = T0H;
            tx_items[i].level1 = 0;
            tx_items[i].duration1 = T0L;
        }
    }

    rmt_write_items(RMT_TX_CHANNEL, tx_items, 8, true);

    escPinRx();
}

// ------------------------------------------------
// GPIO ISR (ESC response decoder)
// ------------------------------------------------

void IRAM_ATTR EscPassthrough::gpio_isr(void *arg)
{
    uint32_t now = micros();
    processEdge(now);
}

void IRAM_ATTR EscPassthrough::processEdge(uint32_t now)
{
    uint32_t dt = now - _lastEdge;
    _lastEdge = now;

    if (dt > 100)
    {
        _rxBitIndex = 0;
        _rxByte = 0;
        return;
    }

    if (dt > 12)
        _rxByte |= (1 << _rxBitIndex);

    _rxBitIndex++;

    if (_rxBitIndex == 8)
    {
        uint16_t next = (_rxHead + 1) % RX_BUF_SIZE;

        if (next != _rxTail)
        {
            _rxBuf[_rxHead] = _rxByte;
            _rxHead = next;
        }

        _rxBitIndex = 0;
        _rxByte = 0;
    }
}

// ------------------------------------------------
// Ring buffer
// ------------------------------------------------

bool EscPassthrough::rxAvailable()
{
    return _rxHead != _rxTail;
}

uint8_t EscPassthrough::rxRead()
{
    uint8_t v = _rxBuf[_rxTail];
    _rxTail = (_rxTail + 1) % RX_BUF_SIZE;
    return v;
}

// ------------------------------------------------
// Update loop
// ------------------------------------------------

bool EscPassthrough::update()
{
    if (!_active)
        return false;

    bool activity = false;

    // Betaflight trick: send bursts without switching RX each byte
    if (Serial.available())
    {
        escPinTx();

        while (Serial.available())
        {
            uint8_t b = Serial.read();
            sendByte(b);
            activity = true;
        }

        escPinRx();
    }

    while (rxAvailable())
    {
        Serial.write(rxRead());
        activity = true;
    }

    return activity;
}

bool EscPassthrough::isActive()
{
    return _active;
}