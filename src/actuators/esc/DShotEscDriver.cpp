/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "actuators/esc/DShotEscDriver.h"
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

bool DShotEscDriver::init()
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
        config.gpio_num = (gpio_num_t)_pin;
        config.clk_div = 1;
        config.mem_block_num = 1;
        config.tx_config.loop_en = true; // <--- Enable continuous looping
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
        err = rmt_driver_install(_rmtChannel, 256, 0);
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
    writeIdle();
    ESP_LOGI(TAG, "DShot ESC ready (rate=%u)", static_cast<uint16_t>(_rate));

    rmt_write_items(_rmtChannel, _rmt_packet, 17, false);
    // 3. FORCE START: Some ESP32 versions need an explicit start command
    // to begin the loop if the first write was non-blocking.
    rmt_tx_start(_rmtChannel, true);

    return true;
}

bool DShotEscDriver::setConfig(ESCConfig &config)
{
    if (config.minPulseUs >= config.maxPulseUs)
        return false;

    if (config.minPulseUs < 800 || config.maxPulseUs > 2200)
        return false;

    if (config.minSpinPercentage < 0 || config.minSpinPercentage > 20)
        return false;

    _config = config;

    writeIdle();
    return true;
}

void DShotEscDriver::writeThrottle(float percent)
{
    if (!_initialized)
        return;

    percent = constrain(percent, 0.0f, 100.0f);
    if (percent > 0 && percent < _config.minSpinPercentage)
        percent = _config.minSpinPercentage;

    uint16_t value = (percent <= 0.0f) ? 0 : (uint16_t)(percent * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) / 100.0f);

    uint16_t packet = makePacket(value, false);

    sendPacket(packet);
}

void DShotEscDriver::writeIdle()
{

    uint16_t packet = makePacket(0, false);
    sendPacket(packet);
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
    for (int i = 0; i < 16; i++)
    {
        bool bit = (packet & (1 << (15 - i))) != 0;
        _rmt_packet[i].level0 = 1;
        _rmt_packet[i].duration0 = bit ? _t1HighTicks : _t0HighTicks;
        _rmt_packet[i].level1 = 0;
        _rmt_packet[i].duration1 = bit ? _t1LowTicks : _t0LowTicks;
    }

    // The Gap / Reset period
    _rmt_packet[16].level0 = 0;
    _rmt_packet[16].duration0 = (uint16_t)_gapTicks;
    _rmt_packet[16].level1 = 0;
    _rmt_packet[16].duration1 = 0; // Crucial: 0 duration signals end of frame

    // Push to hardware
    rmt_write_items(_rmtChannel, _rmt_packet, 17, false);
}

void DShotEscDriver::computeTimings()
{
    uint32_t bitTimeNs;
    uint32_t targetFrameUs = 83; // 1/12000Hz ≈ 83.33 microseconds

    switch (_rate)
    {
    case DShotRate::DSHOT300:
        bitTimeNs = 3333;
        break;
    case DShotRate::DSHOT600:
        bitTimeNs = 1666;
        break;
    case DShotRate::DSHOT1200:
        bitTimeNs = 833;
        break;
    default:
        bitTimeNs = 3333;
        break;
    }

    // RMT tick = 12.5 ns (at 80 MHz APB)
    uint32_t ticksPerBit = (bitTimeNs * 80) / 1000;

    _t0HighTicks = ticksPerBit * 3 / 8;
    _t0LowTicks = ticksPerBit - _t0HighTicks;
    _t1HighTicks = ticksPerBit * 6 / 8;
    _t1LowTicks = ticksPerBit - _t1HighTicks;

    // Calculate Gap: Target (83us) - Data (16 bits * bitTime)
    uint32_t dataDurationUs = (bitTimeNs * 16) / 1000;
    if (dataDurationUs < targetFrameUs)
    {
        uint32_t rawGap = (targetFrameUs - dataDurationUs) * 80;
        // RMT duration is a 15-bit field (max 32767)
        _gapTicks = (rawGap > 32767) ? 32767 : rawGap;
    }
    else
    {
        // For DShot150, the packet is ~106us, so we just use a minimum reset gap
        _gapTicks = 1600; // Minimum 20us reset
    }
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

void DShotEscDriver::deinit()
{
    if (!_initialized)
        return;

    ESP_LOGI(TAG, "Deinitialization DShot on pin %d", _pin);

    // 1. Force the RMT to stop looping immediately
    // This prevents the hardware from trying to read the buffer while we delete the driver
    rmt_tx_stop(_rmtChannel);

    // 2. Set pin to a safe state
    digitalWrite(_pin, LOW);
    pinMode(_pin, INPUT);

    // 3. Uninstall the driver
    // This frees the internal ring buffers and heap memory
    esp_err_t err = rmt_driver_uninstall(_rmtChannel);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to uninstall RMT driver: %s", esp_err_to_name(err));
    }

    // 4. Reset management flags
    rmtChannelUsed[_rmtChannel] = false;
    _initialized = false;

    ESP_LOGI(TAG, "DShot deinitialized safely.");
}
