/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "sensors/RpmSensor.h"
#include <esp_log.h>

const char *RpmSensor::TAG = "RpmSensor";

/* ========= Static ISR-shared data ========= */

RpmSensor *RpmSensor::_instance = nullptr;

volatile uint32_t RpmSensor::_pulseCount = 0;
volatile uint32_t RpmSensor::_lastPulseTimeUs = 0;
volatile uint32_t RpmSensor::_pulseIntervalUs = 0;

RpmSensor::~RpmSensor()
{
    /* Detach interrupt if this instance is active */
    if (_instance == this)
    {
        detachInterrupt(digitalPinToInterrupt(_pin));
        _instance = nullptr;
    }

    /* Free buffers */
    if (_periodBuffer)
    {
        delete[] _periodBuffer;
        _periodBuffer = nullptr;
    }

    if (_averageBuffer)
    {
        delete[] _averageBuffer;
        _averageBuffer = nullptr;
    }

    /* Reset state */
    _rpm = 0.0f;
    setDataValid(false);
    setState(SensorState::SENSOR_UNINITIALIZED);
}

/* ========= Public API ========= */

bool RpmSensor::begin(uint8_t pin,
                      const RpmSensorConfig &config)
{
    if (isReady())
    {
        if (_lastPulseTimeUs == 0 && millis() > 2000)
        {
            ESP_LOGW(TAG, "RPM READY but no pulses → auto-reset");
        }
        else
        {
            ESP_LOGI(TAG, "RPM Sensor READY → reinitializing");
        }

        reset(true); // ALWAYS force re-arm
        return true;
    }

    _pin = pin;
    ESP_LOGI(TAG, "Initialize RPM Sensor on pin: %d ...", pin);
    setState(SensorState::SENSOR_INITIALIZING);

    _instance = this;

    _config = config;
    _config.pulsesPerRevolution = _config.pulsesPerRevolution > 0 ? _config.pulsesPerRevolution : 1;

    _rpm = 0.0f;
    setDataValid(false);

    /* Allocate buffers (instance-owned) */
    _periodBuffer = new uint32_t[_config.periodBufferSize]();
    _averageBuffer = new uint32_t[_config.averageBufferSize]();

    if (!_periodBuffer || !_averageBuffer)
    {
        ESP_LOGE(TAG, "Buffer allocation failed");
        setState(SensorState::SENSOR_ERROR);
        incrementError();
        return false;
    }

    _periodBufferIndex = 0;
    _averageBufferIndex = 0;
    _thresholdPeriod = 0;

    /* Reset ISR data */
    noInterrupts();
    _pulseCount = 0;
    _lastPulseTimeUs = 0;
    _pulseIntervalUs = 0; // prevents a phantom RPM spike after reset.
    interrupts();

    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), pulseCounter, FALLING);

    ESP_LOGI(TAG,
             "RPM Sensor pin=%d PPR=%d avgBuf=%d periodBuf=%d",
             _pin,
             _config.pulsesPerRevolution,
             _config.averageBufferSize,
             _config.periodBufferSize);

    setState(SensorState::SENSOR_READY);
    ESP_LOGI(TAG, "RPM Sensor initialized.");
    return true;
}

void RpmSensor::reset(bool detachIrq)
{
    ESP_LOGI(TAG, "Reset RPM Sensor");

    if (detachIrq)
        detachInterrupt(digitalPinToInterrupt(_pin));

    noInterrupts();
    _pulseCount = 0;
    _lastPulseTimeUs = 0;
    _pulseIntervalUs = 0;
    interrupts();

    _rpm = 0.0f;
    _thresholdPeriod = 0;

    if (_periodBuffer)
        memset(_periodBuffer, 0, sizeof(uint32_t) * _config.periodBufferSize);

    if (_averageBuffer)
        memset(_averageBuffer, 0, sizeof(uint32_t) * _config.averageBufferSize);

    _periodBufferIndex = 0;
    _averageBufferIndex = 0;

    setDataValid(false);
    _updateCount = 0;

    if (detachIrq)
        attachInterrupt(digitalPinToInterrupt(_pin), pulseCounter, FALLING);

    setState(SensorState::SENSOR_READY);
}

bool RpmSensor::update()
{
    if (!isReady())
        return false;

    uint32_t intervalUs;
    uint32_t lastPulseUs;

    /* Copy ISR data atomically */
    noInterrupts();
    intervalUs = _pulseIntervalUs;
    lastPulseUs = _lastPulseTimeUs;
    interrupts();

    uint32_t nowMs = millis();

    /* Timeout → RPM = 0 */
    if (lastPulseUs == 0 ||
        (nowMs - (lastPulseUs / 1000)) > _config.timeoutMs)
    {
        _rpm = 0.0f;
        _thresholdPeriod = 0;

        memset(_periodBuffer, 0, sizeof(uint32_t) * _config.periodBufferSize);
        memset(_averageBuffer, 0, sizeof(uint32_t) * _config.averageBufferSize);

        _periodBufferIndex = 0;
        _averageBufferIndex = 0;

        setDataValid(true);
        ++_updateCount;
        return true;
    }

    /* New interval available */
    if (intervalUs > 0)
        processNewInterval(intervalUs);

    /* Compute average RPM */
    uint32_t minInterval = UINT32_MAX;
    uint32_t maxInterval = 0;
    float sum = 0.0f;
    uint8_t count = 0;

    for (uint8_t i = 0; i < _config.averageBufferSize; i++)
    {
        uint32_t v = _averageBuffer[i];
        if (v > 0)
        {
            sum += v;
            count++;
            minInterval = min(minInterval, v);
            maxInterval = max(maxInterval, v);
        }
    }

    if (count >= 3)
    {
        sum -= (minInterval + maxInterval);
        float avgInterval = sum / (count - 2);

        if (avgInterval >= 500) // noise guard
        {
            _rpm = (60.0f * 1e6f) /
                   (avgInterval * _config.pulsesPerRevolution);
        }
    }

    setDataValid(true);
    ++_updateCount;
    return true;
}

float RpmSensor::getRPM() const
{
    return _rpm;
}

uint32_t RpmSensor::getPulseCount(bool resetCounter)
{
    if (!isReady())
        return 0;

    noInterrupts();
    uint32_t count = _pulseCount;
    if (resetCounter)
        _pulseCount = 0;
    interrupts();

    return count;
}

/* ========= Processing (non-ISR) ========= */

bool RpmSensor::processNewInterval(uint32_t interval)
{
    /* Debounce */
    if (interval <= _config.debounceUs)
        return false;

    /* Dirty buffer */
    _periodBuffer[_periodBufferIndex] = interval;
    _periodBufferIndex =
        (_periodBufferIndex + 1) % _config.periodBufferSize;

    /* Compute outlier threshold */
    uint32_t maxPeriod = 0;
    for (uint8_t i = 0; i < _config.periodBufferSize; i++)
        maxPeriod = max(maxPeriod, _periodBuffer[i]);

    _thresholdPeriod = (uint32_t)(maxPeriod * _config.outlierThreshold);

    if (_thresholdPeriod > 0 && interval < _thresholdPeriod)
        return false;

    /* Clean buffer */
    _averageBuffer[_averageBufferIndex] = interval;
    _averageBufferIndex =
        (_averageBufferIndex + 1) % _config.averageBufferSize;

    return true;
}

/* ========= ISR ========= */

void IRAM_ATTR RpmSensor::pulseCounter()
{
    if (!_instance)
        return;

    uint32_t now = micros();
    uint32_t last = _lastPulseTimeUs;

    _lastPulseTimeUs = now;

    if (last == 0)
        return;

    _pulseIntervalUs = now - last;
    _pulseCount++;
}
