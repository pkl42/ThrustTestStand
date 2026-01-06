#include "HardwareEstop.h"
#include <esp_log.h>

static const char *TAG = "HW_ESTOP";

HardwareEstop::HardwareEstop(uint8_t gpioPin, bool activeLow)
    : _pin(gpioPin),
      _activeLow(activeLow)
{
}

void HardwareEstop::begin()
{
    pinMode(_pin, _activeLow ? INPUT_PULLUP : INPUT_PULLDOWN);

    attachInterruptArg(
        digitalPinToInterrupt(_pin),
        &HardwareEstop::isrHandler,
        this,
        CHANGE // reacts to press + wire break
    );

    ESP_LOGI(TAG, "Hardware E-STOP initialized on GPIO %d", _pin);
}

void HardwareEstop::attachActuator(BaseActuator *actuator)
{
    if (actuator)
        _actuators.push_back(actuator);
}

bool HardwareEstop::isTriggered() const
{
    return _triggered;
}

void IRAM_ATTR HardwareEstop::isrHandler(void *arg)
{
    static_cast<HardwareEstop *>(arg)->handleInterrupt();
}

void IRAM_ATTR HardwareEstop::handleInterrupt()
{
    bool level = digitalRead(_pin);
    bool active = _activeLow ? (level == LOW) : (level == HIGH);

    if (active)
        _triggered = true;
}
