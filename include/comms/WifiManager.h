/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef WIFIMANAGER_H
#define WIFIMANAGER_H

#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "event/EventBus.h"

struct NetworkConfig
{
    String ssid;
    String password;
    IPAddress ip;
    IPAddress gateway;
    IPAddress subnet;
    uint8_t channel;
};

enum class STAState
{
    IDLE,
    CONNECTING,
    CONNECTED,
    FAILED,
    DISCONNECTED
};

static const char *STAStateToString(STAState state)
{
    switch (state)
    {
    case STAState::IDLE:
        return "idle";

    case STAState::CONNECTING:
        return "connecting";

    case STAState::CONNECTED:
        return "connected";

    case STAState::FAILED:
        return "failed";

    case STAState::DISCONNECTED:
        return "disconnected";

    default:
        return "unknown";
    }
}

namespace WiFiManagerEvents
{
    enum StateChangeBits : uint16_t
    {
        CHANGE_STA = 1 << 0,
        CHANGE_AP = 1 << 1
    };
}

class WiFiManager
{
public:
    WiFiManager(uint8_t resetPin,
                const char *default_ssid,
                const char *default_password,
                IPAddress default_ip,
                IPAddress default_gateway,
                IPAddress default_subnet);

    void begin();
    void update();

    NetworkConfig getAPConfig() const { return _netAccessPoint; };
    NetworkConfig getSTAPConfig() const { return _netStation; };

    void saveAPConfig(const NetworkConfig &cfg);
    void saveSTAConfig(const NetworkConfig &cfg);

    void reconnectSTA();

    void writeNetworkConfig(JsonDocument &doc);

    uint8_t findBestChannel();

private:
    void startAP();
    bool startMDNS(const char *hostname);

    void saveConfig(const char *name, const NetworkConfig &cfg);

    void loadConfig(bool forceFactory = false);

    bool isFactoryReset();

    Preferences _prefs;
    NetworkConfig _netFactory;
    NetworkConfig _netAccessPoint;
    NetworkConfig _netStation;

    uint8_t _resetPin;

    STAState _staState = STAState::IDLE;
    STAState _lastSTAState;
    void checkStateChanges();
    unsigned long _connectStart = 0;
};

#endif // WIFIMANAGER_H