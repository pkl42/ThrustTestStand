/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "sensors/RpmSensor.h"
#include <esp_log.h>

const char *RpmSensor::TAG = "RpmSensor";

/* ========= Lifecycle ========= */

RpmSensor::~RpmSensor()
{
    deinit();
}

/* ========= Initialization ========= */

bool RpmSensor::begin(uint8_t pin, const RpmUiConfig &config)
{
    if (isReady())
    {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }

    _pin = pin;

    setState(SensorState::SENSOR_INITIALIZING);

    // Apply config BEFORE init
    _uiConfig = config;
    _config = convertUIConfigToRuntime(config);

    if (_config.pulsesPerRevolution == 0)
        _config.pulsesPerRevolution = 1;

    ESP_LOGI(TAG, "Init PCNT RPM on pin %d", _pin);

    if (!initPCNT())
    {
        ESP_LOGE(TAG, "PCNT init failed");
        setState(SensorState::SENSOR_ERROR);
        incrementError();
        return false;
    }

    reset();

    setState(SensorState::SENSOR_READY);
    ESP_LOGI(TAG, "RPM Sensor (PCNT) ready");

    return true;
}

/* ========= Config Handling ========= */

bool RpmSensor::setUIConfig(const RpmUiConfig &uiConfig)
{
    if (_uiConfig == uiConfig)
        return false;

    RpmSensorConfig newCfg = convertUIConfigToRuntime(uiConfig);

    bool filterChanged =
        (newCfg.filterCycles != _config.filterCycles);

    bool timingChanged =
        (newCfg.sampleTimeMs != _config.sampleTimeMs);

    bool smoothingChanged =
        (newCfg.alpha != _config.alpha);

    bool pprChanged =
        (newCfg.pulsesPerRevolution != _config.pulsesPerRevolution);

    _uiConfig = uiConfig;
    _config = newCfg;

    if (!isReady())
        return true;

    // --- Apply runtime changes ---
    if (filterChanged)
    {
        ESP_LOGI(TAG, "Updating PCNT filter");
        applyPcntFilter();
    }

    if (timingChanged || smoothingChanged || pprChanged)
    {
        ESP_LOGI(TAG, "Resetting filter state due to config change");
        reset(); // ensures stable transition
    }

    return true;
}

/* ========= Conversion ========= */

RpmSensor::RpmSensorConfig
RpmSensor::convertUIConfigToRuntime(const RpmUiConfig &ui)
{
    RpmSensorConfig cfg;

    cfg.pulsesPerRevolution = ui.pulsesPerRevolution;
    cfg.sampleTimeMs = rpmResponseToSampleTime(ui.responseMode);
    cfg.filterCycles = rpmFilterToCycles(ui.filterLevel);
    cfg.alpha = rpmSmoothingToAlpha(ui.smoothing);
    cfg.timeoutMs = 2000;

    return cfg;
}

/* ========= PCNT Setup ========= */

bool RpmSensor::initPCNT()
{
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = _pin;
    pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = _pcntUnit;

    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DIS;

    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_KEEP;

    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = 0;

    if (pcnt_unit_config(&pcnt_config) != ESP_OK)
        return false;

    applyPcntFilter();

    pcnt_counter_pause(_pcntUnit);
    pcnt_counter_clear(_pcntUnit);
    pcnt_counter_resume(_pcntUnit);

    return true;
}

void RpmSensor::applyPcntFilter()
{
    pcnt_counter_pause(_pcntUnit);

    pcnt_set_filter_value(_pcntUnit, _config.filterCycles);
    pcnt_filter_enable(_pcntUnit);

    pcnt_counter_resume(_pcntUnit);
}

void RpmSensor::deinitPCNT()
{
    pcnt_counter_pause(_pcntUnit);
    pcnt_counter_clear(_pcntUnit);
    pcnt_filter_disable(_pcntUnit);
}

/* ========= Deinit ========= */

void RpmSensor::deinit()
{
    if (!isReady())
        return;

    ESP_LOGI(TAG, "Deinit RPM Sensor (PCNT)");

    deinitPCNT();

    setState(SensorState::SENSOR_UNINITIALIZED);
}

/* ========= Reset ========= */

void RpmSensor::reset()
{
    if (!isReady())
        return;

    pcnt_counter_pause(_pcntUnit);
    pcnt_counter_clear(_pcntUnit);
    pcnt_counter_resume(_pcntUnit);

    _lastSampleTimeMs = millis();
    _lastPulseTimeMs = millis();

    _rpm = 0.0f;
    _rpmFiltered = 0.0f;

    setDataValid(false);
    _updateCount = 0;
}

/* ========= Update ========= */

bool RpmSensor::update()
{
    if (!isReady())
        return false;

    uint32_t now = millis();
    uint32_t dt = now - _lastSampleTimeMs;

    if (dt < _config.sampleTimeMs)
        return false;

    int16_t count = 0;
    pcnt_get_counter_value(_pcntUnit, &count);
    pcnt_counter_clear(_pcntUnit);

    _lastSampleTimeMs = now;

    if (count > 0)
        _lastPulseTimeMs = now;

    if ((now - _lastPulseTimeMs) > _config.timeoutMs)
    {
        _rpm = 0.0f;
        _rpmFiltered = 0.0f;
        setDataValid(true);
        ++_updateCount;
        return true;
    }

    float dtSeconds = dt / 1000.0f;

    float rpmRaw =
        (count * 60.0f) /
        (dtSeconds * _config.pulsesPerRevolution);

    _rpmFiltered =
        (1.0f - _config.alpha) * _rpmFiltered +
        _config.alpha * rpmRaw;

    _rpm = _rpmFiltered;

    setDataValid(true);
    ++_updateCount;
    return true;
}

/* ========= Data Access ========= */

float RpmSensor::getRPM() const
{
    return _rpm;
}

uint32_t RpmSensor::getPulseCount(bool resetCounter)
{
    if (!isReady())
        return 0;

    int16_t count = 0;
    pcnt_get_counter_value(_pcntUnit, &count);

    if (resetCounter)
        pcnt_counter_clear(_pcntUnit);

    return (uint32_t)count;
}