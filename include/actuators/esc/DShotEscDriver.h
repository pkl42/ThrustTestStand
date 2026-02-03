/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSHOT_ESC_DRIVER_H
#define DSHOT_ESC_DRIVER_H

#include "Arduino.h"
#include <driver/rmt.h>
#include "actuators/esc/EscSignalDriver.h"

static bool isDShot(EscDriverType t)
{
    return t == EscDriverType::ESC_DRIVER_DSHOT150 ||
           t == EscDriverType::ESC_DRIVER_DSHOT300 ||
           t == EscDriverType::ESC_DRIVER_DSHOT600;
}

/**
 * @class DShotEscDriver
 * @brief DShot ESC driver using ESP32 RMT peripheral.
 *
 * Sends throttle commands as digital DShot packets.
 * Supports DShot150 / DShot300 / DShot600.
 */
class DShotEscDriver : public EscSignalDriver
{
public:
    enum class DShotRate : uint16_t
    {
        DSHOT150 = 150,
        DSHOT300 = 300,
        DSHOT600 = 600
    };

    DShotEscDriver(uint8_t pin,
                   DShotRate rate = DShotRate::DSHOT300);

    bool begin() override;
    void writeThrottle(float percent) override;
    void writeStop() override;

    uint16_t getMinPulseUs() const override { return _minPulseUsEquivalent; }
    uint16_t getMaxPulseUs() const override { return _maxPulseUsEquivalent; }

    bool setPulseRangeUs(uint16_t minUs, uint16_t maxUs) override;

    bool setRate(DShotRate rate);
    DShotRate rate() const { return _rate; }

    void stop() override;

    void update() override;

    static DShotRate toDShotRate(EscDriverType t)
    {
        switch (t)
        {
        case EscDriverType::ESC_DRIVER_DSHOT150:
            return DShotEscDriver::DShotRate::DSHOT150;
        case EscDriverType::ESC_DRIVER_DSHOT300:
            return DShotEscDriver::DShotRate::DSHOT300;
        case EscDriverType::ESC_DRIVER_DSHOT600:
            return DShotEscDriver::DShotRate::DSHOT600;
        default:
            return DShotEscDriver::DShotRate::DSHOT300;
        }
    }

private:
    void sendPacket(uint16_t packet);
    uint16_t makePacket(uint16_t value, bool telemetry);

    void computeTimings();

    uint8_t _pin;
    DShotRate _rate;

    static bool rmtChannelUsed[RMT_CHANNEL_MAX];
    bool _channelFound = false;
    uint16_t _currentPacket;

    rmt_channel_t _rmtChannel;
    uint32_t _t0HighTicks;
    uint32_t _t0LowTicks;
    uint32_t _t1HighTicks;
    uint32_t _t1LowTicks;

    static const uint16_t MIN_COMMAND = 48;
    static const uint16_t MAX_COMMAND = 2047;

    uint16_t _minPulseUsEquivalent = 1000;
    uint16_t _maxPulseUsEquivalent = 2000;
};

#endif // DSHOT_ESC_DRIVER_H
