#ifndef STATUS_LED_H
#define STATUS_LED_H
#include <stdint.h>

class StatusLed
{
public:
    enum class State
    {
        OFF,
        READY,   // green steady
        BUSY,    // yellow steady or slow blink
        RUNNING, // green blinking
        WARNING, // yellow blinking
        ERROR    // red blinking
    };

    void begin();
    void setState(State state);
    void update(); // must be called periodically (for blinking)

private:
    State _state = State::OFF;
    uint32_t _lastToggle = 0;
    bool _ledOn = true;

    void applyColor(bool on);
};

#endif // STATUS_LED_H
