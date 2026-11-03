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
    return t == EscDriverType::ESC_DRIVER_DSHOT1200 ||
           t == EscDriverType::ESC_DRIVER_DSHOT300 ||
           t == EscDriverType::ESC_DRIVER_DSHOT600;
}

/**
 * @class DShotEscDriver
 * @brief DShot ESC driver using the ESP32 RMT peripheral.
 *
 * This driver generates DShot digital throttle signals using the ESP32's
 * RMT (Remote Control) hardware for precise timing.
 *
 * Supported protocols:
 * - DSHOT300
 * - DSHOT600
 * - DSHOT1200
 *
 * @note
 * - Output is fully digital (no PWM), timing is critical.
 * - Uses one RMT channel per instance.
 * - Not all channels may be available (see @ref init()).
 *
 * @warning
 * - writeThrottle() is executed synchronously and should not be called
 *   from time-critical contexts unless timing is well understood.
 * - No internal safety checks for motor state are enforced.
 *
 * @see EscSignalDriver
 */
class DShotEscDriver : public EscSignalDriver
{
public:
    /**
     * @enum DShotRate
     * @brief Supported DShot signaling rates (kbit/s).
     */
    enum class DShotRate : uint16_t
    {
        DSHOT300 = 300,
        DSHOT600 = 600,
        DSHOT1200 = 1200
    };

    /**
     * @brief Construct a new DShot ESC driver.
     * @param pin GPIO pin used for signal output
     * @param rate Initial DShot rate
     */
    DShotEscDriver(uint8_t pin,
                   DShotRate rate = DShotRate::DSHOT300);

    /**
     * @brief Initialize the RMT peripheral and allocate a channel.
     * @return true if initialization succeeded
     *
     * @note Fails if no RMT channel is available.
     */
    bool init() override;

    /**
     * @brief Send a throttle command to the ESC.
     * @param percent Throttle value in range [0.0, 100.0]
     *
     * Internally converted to DShot value range
     * [@ref DSHOT_MIN_THROTTLE, @ref DSHOT_MAX_THROTTLE].
     */
    void writeThrottle(float percent) override;

    /**
     * @brief Send an idle (zero throttle) command.
     */
    void writeIdle() override;

    /**
     * @brief Apply ESC configuration.
     */
    bool setConfig(ESCConfig &config) override;

    /**
     * @brief Change the DShot rate.
     * @return true if successfully applied
     *
     * Recomputes internal timing parameters.
     */
    bool setRate(DShotRate rate);

    /**
     * @brief Get current DShot rate.
     */
    DShotRate rate() const { return _rate; }

    /**
     * @brief Release RMT resources.
     */
    void deinit() override;

    /**
     * @brief Convert generic driver type to DShot rate.
     */
    static DShotRate toDShotRate(EscDriverType t)
    {
        switch (t)
        {
        case EscDriverType::ESC_DRIVER_DSHOT300:
            return DShotEscDriver::DShotRate::DSHOT300;
        case EscDriverType::ESC_DRIVER_DSHOT600:
            return DShotEscDriver::DShotRate::DSHOT600;
        case EscDriverType::ESC_DRIVER_DSHOT1200:
            return DShotEscDriver::DShotRate::DSHOT1200;
        default:
            return DShotEscDriver::DShotRate::DSHOT300;
        }
    }

private:
    /**
     * @brief Send a prepared DShot packet via RMT.
     */
    void sendPacket(uint16_t packet);

    /**
     * @brief Create a 16-bit DShot packet including checksum.
     */
    uint16_t makePacket(uint16_t value, bool telemetry);

    /**
     * @brief Compute timing ticks for current DShot rate.
     */
    void computeTimings();

    uint8_t _pin;    ///< Output GPIO
    DShotRate _rate; ///< Active DShot rate

    static bool rmtChannelUsed[RMT_CHANNEL_MAX]; ///< Global channel usage tracking
    bool _channelFound = false;

    volatile uint16_t _currentPacket;

    rmt_channel_t _rmtChannel;

    // Timing values (RMT ticks)
    uint32_t _t0HighTicks;
    uint32_t _t0LowTicks;
    uint32_t _t1HighTicks;
    uint32_t _t1LowTicks;

    /**
     * @brief Minimum valid DShot throttle value.
     *
     * Values below this are interpreted as special commands.
     */
    static const uint16_t DSHOT_MIN_THROTTLE = 48;

    /**
     * @brief Maximum DShot throttle value.
     */
    static const uint16_t DSHOT_MAX_THROTTLE = 2047;

    rmt_item32_t _rmt_packet[17]; ///< Preallocated RMT buffer (16 bits + reset)

    /**
     * @brief Convert packet into RMT waveform items.
     */
    void prepareRmtItem(uint16_t packet);

    uint32_t _gapTicks; ///< Inter-frame gap timing
};
#endif // DSHOT_ESC_DRIVER_H
