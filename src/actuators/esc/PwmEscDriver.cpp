/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "actuators/esc/PwmEscDriver.h"
#include <esp_log.h>
#include "driver/ledc.h"

static const char *TAG = "PwmEscDriver";

PwmEscDriver::PwmEscDriver(uint8_t pin, uint16_t freq, uint8_t resolution, uint8_t channel)
    : _pin(pin), _freq(freq), _resolution(resolution), _channel(channel)
{
}

bool PwmEscDriver::begin()
{
    if (_initialized)
        return _initialized;
    _initialized = false;
    if (_freq <= 50 && _resolution > 13)
    {
        ESP_LOGW(TAG, "Resolution too high for %d Hz, clamping to 13 bits", _freq);
        _resolution = 13;
    }

    _maxDuty = (1 << _resolution) - 1;

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)_resolution,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = _freq,
        .clk_cfg = LEDC_AUTO_CLK};

    if (ledc_timer_config(&timer) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC timer");
        return _initialized;
    }

    ledc_channel_config_t channelCfg = {
        .gpio_num = _pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};

    if (ledc_channel_config(&channelCfg) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC channel");
        return _initialized;
    }

    writeStop();
    ESP_LOGI(TAG, "PWM ESC ready on pin %d (%d Hz)", _pin, _freq);
    _initialized = true;
    return _initialized;
}

void PwmEscDriver::writeThrottle(float percent)
{
    percent = constrain(percent, 0.0f, 100.0f);
    uint16_t pulse = map(percent * 100, 0, 10000, _minPulseUs, _maxPulseUs);
    setPulseWidth(pulse);
}

void PwmEscDriver::writeStop()
{
    if (_initialized)
        setPulseWidth(_minPulseUs);
}

bool PwmEscDriver::setPulseRangeUs(uint16_t minUs, uint16_t maxUs)
{
    if (minUs >= maxUs)
        return false;

    if (minUs < 800 || maxUs > 2200)
        return false;

    _minPulseUs = minUs;
    _maxPulseUs = maxUs;

    writeStop();
    return true;
}

void PwmEscDriver::setPulseWidth(uint16_t pulseUs)
{
    pulseUs = constrain(pulseUs, _minPulseUs, _maxPulseUs);
    uint32_t periodUs = 1000000 / _freq;
    float dutyRatio = (float)pulseUs / periodUs;
    uint32_t duty = dutyRatio * _maxDuty;
    duty = constrain(duty, 0, _maxDuty);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel);
}

void PwmEscDriver::stop()
{
    if (_initialized)
    {
        ESP_LOGI(TAG, "stop");
        writeStop();
        _initialized = false;
    }
}
