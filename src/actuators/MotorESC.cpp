/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "actuators/MotorESC.h"
#include <esp_log.h>
#include "driver/ledc.h"

static const char *TAG = "MotorESC";

MotorESC::MotorESC(uint8_t pin,
                   uint16_t freq,
                   uint8_t resolution,
                   uint8_t channel)
    : BaseActuator(TAG),
      _pin(pin),
      _channel(channel),
      _freq(freq),
      _resolution(resolution),
      _escState(EscState::DISARMED)
{
}

bool MotorESC::begin()
{
    setState(ActuatorState::ACTU_INITIALIZING);
    _escState = EscState::DISARMED;

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
        setState(ActuatorState::ACTU_ERROR);
        return false;
    }

    ledc_channel_config_t channel = {
        .gpio_num = _pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};

    if (ledc_channel_config(&channel) != ESP_OK)
    {
        setState(ActuatorState::ACTU_ERROR);
        return false;
    }

    forceStopHardware(); // ensures min throttle
    setState(ActuatorState::ACTU_STOPPED);

    ESP_LOGI(TAG, "ESC ready on pin %d (%dHz) for arming", _pin, _freq);
    return true;
}

bool MotorESC::arm(uint16_t armTimeMs)
{
    if (!isReady())
        return false;

    if (_escState != EscState::DISARMED)
        return false;

    ESP_LOGI(TAG, "Starting ESC arming sequence armTimeMs: %u _minPulseUs: %u", armTimeMs, _minPulseUs);

    // Force minimum throttle
    setPulseWidth(_minPulseUs);

    _armStartTime = millis();
    _armDuration = armTimeMs;
    _escState = EscState::ARMING;
    return true;
}

bool MotorESC::disarm()
{
    if (!isReady())
        return false;
    if (_escState != EscState::ARMED)
        return false;

    forceStopHardware();
    return true;
};

bool MotorESC::update()
{
    if (!isReady())
        return false;

    unsigned long now = millis();

    // ---------- ARMING ----------
    if (_escState == EscState::ARMING)
    {
        if (now - _armStartTime >= _armDuration)
        {
            _escState = EscState::ARMED;
            ESP_LOGI(TAG, "ESC armed");
        }
        return true;
    }

    // ---------- RAMP ----------
    if (!_smoothingActive || _escState == EscState::DISARMED)
        return false;

    float progress =
        (float)(now - _stateStartTime) / _transitionTimeMs;

    if (progress >= 1.0f)
    {
        _smoothingActive = false;
        applyThrottle(_targetThrottle);

        if (_targetThrottle <= 0.0f)
        {
            _escState = EscState::ARMED;
            setState(ActuatorState::ACTU_STOPPED);
        }
        else
        {
            _escState = EscState::RUNNING;
            setState(ActuatorState::ACTU_ACTIVE);
        }

        return true;
    }

    float throttle = _startThrottle +
                     (_targetThrottle - _startThrottle) * progress;

    applyThrottle(throttle);
    return true;
}

float MotorESC::setThrottle(float throttlePercent,
                            bool smooth,
                            unsigned long transitionTimeMs)
{
    if (!canActuate() || _escState != EscState::ARMED && _escState != EscState::RUNNING)
        return 0.0f;

    if (smooth)
    {
        _startThrottle = _currentThrottle;
        _targetThrottle = throttlePercent;
        _transitionTimeMs = transitionTimeMs;
        _stateStartTime = millis();
        _smoothingActive = true;

        if (throttlePercent > 0)
            _escState = EscState::RUNNING;

        setState(ActuatorState::ACTU_ACTIVE);
        return _targetThrottle;
    }

    applyThrottle(throttlePercent);

    _targetThrottle = throttlePercent;
    _escState = (throttlePercent > 0.f) ? EscState::RUNNING : EscState::ARMED;

    setState(throttlePercent > 0.f
                 ? ActuatorState::ACTU_ACTIVE
                 : ActuatorState::ACTU_STOPPED);

    return throttlePercent;
}

void MotorESC::setPulseWidth(uint16_t pulseWidthUs)
{
    pulseWidthUs = constrain(pulseWidthUs, _minPulseUs, _maxPulseUs);

    uint32_t periodUs = 1000000 / _freq;
    float dutyRatio = (float)pulseWidthUs / periodUs;
    uint32_t duty = dutyRatio * _maxDuty;

    duty = constrain(duty, 0, _maxDuty);

    ledc_set_duty(LEDC_LOW_SPEED_MODE,
                  (ledc_channel_t)_channel,
                  duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
                     (ledc_channel_t)_channel);
}

void MotorESC::stop()
{
    forceStopHardware();
    _escState = EscState::DISARMED;
    setState(ActuatorState::ACTU_STOPPED);
}

void MotorESC::stopWithError()
{
    forceStopHardware();
    _escState = EscState::DISARMED;
    setState(ActuatorState::ACTU_ERROR);
}

void MotorESC::forceStopHardware()
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE,
                  (ledc_channel_t)_channel,
                  0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
                     (ledc_channel_t)_channel);

    _currentThrottle = 0.0f;
    _targetThrottle = 0.0f;
    _smoothingActive = false;
    _escState = EscState::DISARMED;
}

bool MotorESC::isAtTargetSpeed(float tolerance) const
{
    return fabs(_currentThrottle - _targetThrottle) <= tolerance;
    // for a later version to include rpm
    //    return abs(_measuredRPM - _targetRPM) < _rpmTolerance &&
    //       abs(_currentThrottle - _targetThrottle) < 2.0f;
}

void MotorESC::applyThrottle(float throttlePercent)
{
    uint16_t pulse = map(throttlePercent * 100, 0, 10000,
                         _minPulseUs, _maxPulseUs);

    setPulseWidth(pulse);
    _currentThrottle = throttlePercent;
}

bool MotorESC::setPulseRangeUs(uint16_t minPulseUs, uint16_t maxPulseUs)
{
    if (getState() == ActuatorState::ACTU_ACTIVE)
        return false;

    if (minPulseUs >= maxPulseUs)
        return false;

    if (minPulseUs < 800 || maxPulseUs > 2200)
        return false;

    _minPulseUs = minPulseUs;
    _maxPulseUs = maxPulseUs;

    // Ensure output stays at zero throttle
    applyThrottle(0.0f);

    return true;
}
