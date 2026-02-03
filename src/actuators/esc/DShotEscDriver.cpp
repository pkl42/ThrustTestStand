/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "actuators/esc/DShotEscDriver.h"
#include <driver/rmt.h>
#include <esp_log.h>

static const char *TAG = "DShotEscDriver";

bool DShotEscDriver::rmtChannelUsed[RMT_CHANNEL_MAX];

DShotEscDriver::DShotEscDriver(uint8_t pin,
                               DShotRate rate)
    : _pin(pin),
      _rate(rate)
{
    _rmtChannel = RMT_CHANNEL_MAX; // marker for “not assigned yet”
}

bool DShotEscDriver::begin()
{
    if (_initialized)
    {
        ESP_LOGI(TAG, "ESC already initialized on channel %d", _rmtChannel);
        return true;
    }

    bool assigned = false;

    // Try to auto-assign a free channel
    for (int ch = 0; ch < RMT_CHANNEL_MAX; ++ch)
    {
        if (rmtChannelUsed[ch])
            continue; // already used by another ESC instance

        _rmtChannel = static_cast<rmt_channel_t>(ch);

        // Configure timings
        computeTimings();

        rmt_config_t config = {};
        config.rmt_mode = RMT_MODE_TX;
        config.channel = _rmtChannel;
        config.gpio_num = static_cast<gpio_num_t>(_pin);
        config.clk_div = 1;
        config.mem_block_num = 1;
        config.tx_config.loop_en = false;
        config.tx_config.carrier_en = false;
        config.tx_config.idle_output_en = true;
        config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

        // Attempt RMT configuration
        esp_err_t err = rmt_config(&config);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "rmt_config() failed on channel %d: %s, trying next", _rmtChannel, esp_err_to_name(err));
            continue;
        }

        // Attempt driver install
        err = rmt_driver_install(_rmtChannel, 0, 0);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "rmt_driver_install() failed on channel %d: %s, trying next", _rmtChannel, esp_err_to_name(err));
            continue;
        }

        // Success!
        rmtChannelUsed[_rmtChannel] = true;
        _channelFound = true;
        assigned = true;
        ESP_LOGI(TAG, "Auto-assigned RMT channel %d for pin %d", _rmtChannel, _pin);
        break;
    }

    if (!assigned)
    {
        ESP_LOGE(TAG, "No free RMT channels available for pin %d", _pin);
        _channelFound = false;
        return false;
    }

    _initialized = true;
    writeStop();
    ESP_LOGI(TAG, "DShot ESC ready (rate=%u)", static_cast<uint16_t>(_rate));
    return true;
}

void DShotEscDriver::writeThrottle(float percent)
{
    // Clamp percent [0–100]
    percent = constrain(percent, 0.0f, 100.0f);

    // Map percent to min/max equivalent pulse
    // This keeps D-Shot consistent with MotorESC / ThrustStand facade
    float scaledPercent = percent;

    // Optional: if you want to use stored pulse range for scaling
    // scaledPercent = map(percent, 0.0f, 100.0f,
    //                     _minPulseUsEquivalent, _maxPulseUsEquivalent);

    // Convert to D-Shot command value
    uint16_t value = MIN_COMMAND + scaledPercent * (MAX_COMMAND - MIN_COMMAND) / 100.0f;

    // Build the 16-bit D-Shot packet
    _currentPacket = makePacket(value, false);
}

void DShotEscDriver::writeStop()
{

    _currentPacket = makePacket(MIN_COMMAND, false);
}

uint16_t DShotEscDriver::makePacket(uint16_t value, bool telemetry)
{
    uint16_t packet = (value << 1) | (telemetry ? 1 : 0);

    // Compute checksum
    uint8_t csum = 0;
    uint16_t csum_data = packet;
    for (int i = 0; i < 3; i++)
    {
        csum ^= csum_data & 0xF;
        csum_data >>= 4;
    }

    packet = (packet << 4) | (csum & 0xF);
    return packet;
}

void DShotEscDriver::sendPacket(uint16_t packet)
{
    rmt_item32_t items[16]; // 16 bits per packet
    for (int i = 0; i < 16; i++)
    {
        bool bit = (packet & (1 << (15 - i))) != 0;
        if (bit)
        {
            items[i].duration0 = _t1HighTicks;
            items[i].level0 = 1;
            items[i].duration1 = _t1LowTicks;
            items[i].level1 = 0;
        }
        else
        {
            items[i].duration0 = _t0HighTicks;
            items[i].level0 = 1;
            items[i].duration1 = _t0LowTicks;
            items[i].level1 = 0;
        }
    }

    rmt_write_items((rmt_channel_t)_rmtChannel, items, 16, true);
    // rmt_wait_tx_done((rmt_channel_t)_rmtChannel, portMAX_DELAY);
}

bool DShotEscDriver::setPulseRangeUs(uint16_t minUs, uint16_t maxUs)
{
    if (minUs >= maxUs)
        return false;

    // Save the range for ThrustStand / MotorESC queries
    _minPulseUsEquivalent = minUs;
    _maxPulseUsEquivalent = maxUs;

    return true;
}

void DShotEscDriver::computeTimings()
{
    uint32_t bitTimeNs;

    switch (_rate)
    {
    case DShotRate::DSHOT150:
        bitTimeNs = 6666;
        break;
    case DShotRate::DSHOT300:
        bitTimeNs = 3333;
        break;
    case DShotRate::DSHOT600:
        bitTimeNs = 1666;
        break;
    default:
        bitTimeNs = 3333;
        break;
    }

    // RMT tick = 12.5 ns (80 MHz APB)
    uint32_t ticks = (bitTimeNs * 80) / 1000; // ns → ticks

    _t0HighTicks = ticks * 3 / 8;
    _t0LowTicks = ticks - _t0HighTicks;
    _t1HighTicks = ticks * 6 / 8;
    _t1LowTicks = ticks - _t1HighTicks;
}

bool DShotEscDriver::setRate(DShotRate rate)
{
    // if (_initialized)
    // {
    //     ESP_LOGE(TAG, "Cannot change DShot rate while active");
    //     return false;
    // }

    _rate = rate;
    computeTimings();

    ESP_LOGI(TAG, "DShot rate set to %u", static_cast<uint16_t>(_rate));
    return true;
}

void DShotEscDriver::stop()
{
    if (_initialized)
    {
        ESP_LOGI(TAG, "stop");
        writeStop();
        rmt_driver_uninstall(_rmtChannel);
        rmtChannelUsed[_rmtChannel] = false;
        _channelFound = false;
        _initialized = false;
    }
}

void DShotEscDriver::update()
{
    if (!_initialized)
        return;

    sendPacket(_currentPacket);
}
