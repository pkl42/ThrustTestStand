#include "MotorESC.h"
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
      _state(EscState::DISARMED)
{
}

bool MotorESC::begin()
{
    setState(ActuatorState::ACTU_INITIALIZING);
    _state = EscState::DISARMED;

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
    setState(ActuatorState::ACTU_READY);

    ESP_LOGI(TAG, "ESC ready on pin %d (%dHz)", _pin, _freq);
    return true;
}

void MotorESC::arm(uint16_t armTimeMs)
{
    if (!isReady() || _state != EscState::DISARMED)
        return;

    ESP_LOGI(TAG, "Starting ESC arming sequence armTimeMs: %u _minPulseUs: %u", armTimeMs, _minPulseUs);

    // Force minimum throttle
    setPulseWidth(_minPulseUs);

    _armStartTime = millis();
    _armDuration = armTimeMs;
    _state = EscState::ARMING;
}

bool MotorESC::update()
{
    if (!isReady())
        return false;

    unsigned long now = millis();

    // ---------- ARMING ----------
    if (_state == EscState::ARMING)
    {
        if (now - _armStartTime >= _armDuration)
        {
            _state = EscState::ARMED;
            ESP_LOGI(TAG, "ESC armed");
        }
        return true;
    }

    // ---------- RAMP ----------
    if (!_smoothingActive || _state == EscState::DISARMED)
        return false;

    float progress =
        (float)(now - _stateStartTime) / _transitionDuration;

    if (progress >= 1.0f)
    {
        _smoothingActive = false;
        applyThrottle(_targetThrottle);

        if (_targetThrottle <= 0.0f)
        {
            _state = EscState::ARMED;
            setState(ActuatorState::ACTU_STOPPED);
        }
        else
        {
            _state = EscState::RUNNING;
            setState(ActuatorState::ACTU_ACTIVE);
        }

        return true;
    }

    float throttle = _startThrottle +
                     (_targetThrottle - _startThrottle) * progress;

    applyThrottle(throttle);
    return true;
}

void MotorESC::setMaxThrottlePercent(float maxPercent)
{
    _maxThrottlePercent = constrain(maxPercent, 0.0f, 100.0f);
}

float MotorESC::setThrottle(float throttlePercent,
                            bool smooth,
                            unsigned long accelTimeMs)
{
    if (!canActuate() || _state != EscState::ARMED && _state != EscState::RUNNING)
        return 0.0f;

    throttlePercent = constrain(throttlePercent, 0.0f, _maxThrottlePercent);

    if (smooth)
    {
        _startThrottle = _currentThrottle;
        _targetThrottle = throttlePercent;
        _transitionDuration = accelTimeMs;
        _stateStartTime = millis();
        _smoothingActive = true;

        if (throttlePercent > 0)
            _state = EscState::RUNNING;

        setState(ActuatorState::ACTU_ACTIVE);
        return _targetThrottle;
    }

    applyThrottle(throttlePercent);

    _targetThrottle = throttlePercent;
    _state = (throttlePercent > 0.f) ? EscState::RUNNING : EscState::ARMED;

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
    _state = EscState::DISARMED;
    setState(ActuatorState::ACTU_STOPPED);
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
    _state = EscState::DISARMED;
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
    throttlePercent = constrain(throttlePercent, 0.0f, _maxThrottlePercent);

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
