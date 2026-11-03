/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WiFiManager.h"
#include <ESPmDNS.h>
#include "esp_wifi.h"
#include <esp_log.h>
#include "core/Config.h"

static const char *TAG = "WiFiManager";

WiFiManager::WiFiManager(uint8_t resetPin,
                         const char *default_ssid,
                         const char *default_password,
                         IPAddress default_ip,
                         IPAddress default_gateway,
                         IPAddress default_subnet)
    : _resetPin(resetPin)
{
    _netFactory.ssid = default_ssid;
    _netFactory.password = default_password;
    _netFactory.ip = default_ip;
    _netFactory.gateway = default_gateway;
    _netFactory.subnet = default_subnet;
}

void WiFiManager::begin()
{
    pinMode(_resetPin, INPUT_PULLUP);

    bool forceFactory = isFactoryReset();
    loadConfig(forceFactory);

    WiFi.mode(WIFI_AP_STA);

    _netAccessPoint.channel = findBestChannel();

    startAP();

    // Initialize state machine
    if (_netStation.ssid.isEmpty())
    {
        _staState = STAState::IDLE;
    }
    else
    {
        reconnectSTA(); // 👈 unified entry point
    }
}

void WiFiManager::startAP()
{
    WiFi.softAPConfig(_netAccessPoint.ip,
                      _netAccessPoint.gateway,
                      _netAccessPoint.subnet);

    WiFi.softAP(_netAccessPoint.ssid.c_str(),
                _netAccessPoint.password.c_str(),
                _netAccessPoint.channel);

    // WiFi optimizations
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    esp_wifi_set_max_tx_power(78);

    wifi_second_chan_t second;
    uint8_t primary;

    esp_wifi_get_channel(&primary, &second);

    ESP_LOGI(TAG, "AP started: %s on Channel: %d", _netAccessPoint.ssid.c_str(), primary);
    ESP_LOGI(TAG, "AP IP: %s", WiFi.softAPIP().toString().c_str());
}

void WiFiManager::reconnectSTA()
{
    if (_netStation.ssid.isEmpty())
    {
        ESP_LOGW(TAG, "Reconnect requested but no STA SSID configured → skipping STA connection");
        _staState = STAState::FAILED;
        return;
    }

    ESP_LOGI(TAG, "Connecting STA: %s", _netStation.ssid.c_str());

    WiFi.disconnect(true);
    delay(100);

    WiFi.begin(_netStation.ssid.c_str(),
               _netStation.password.c_str());

    _staState = STAState::CONNECTING;
    _connectStart = millis();
}

void WiFiManager::update()
{
    wl_status_t status = WiFi.status();

    switch (_staState)
    {
    case STAState::CONNECTING:
    {
        if (status == WL_CONNECTED)
        {
            _staState = STAState::CONNECTED;

            ESP_LOGI(TAG, "STA connected: %s",
                     WiFi.localIP().toString().c_str());

            startMDNS("thruststand");
        }
        else if (millis() - _connectStart > 10000)
        {
            _staState = STAState::FAILED;

            ESP_LOGW(TAG, "STA connection timeout");
        }
        break;
    }

    case STAState::CONNECTED:
    {
        if (status != WL_CONNECTED)
        {
            _staState = STAState::DISCONNECTED;

            ESP_LOGW(TAG, "STA disconnected");
        }
        break;
    }

    case STAState::DISCONNECTED:
    {
        // Optional: auto-reconnect
        // reconnectSTA();
        break;
    }

    case STAState::FAILED:
    {
        // stay here until manual reconnect
        break;
    }

    case STAState::IDLE:
    default:
        break;
    }

    // mDNS (only when connected)
    if (_staState == STAState::CONNECTED)
    {
        // MDNS.update();// 2026/03/19 as of legacy library this routine does not yet exists
    }
}

bool WiFiManager::startMDNS(const char *hostname)
{
    MDNS.end();

    if (!MDNS.begin(hostname))
    {
        ESP_LOGE(TAG, "mDNS failed");
        return false;
    }

    MDNS.addService("http", "tcp", 80);

    ESP_LOGI(TAG, "mDNS started: http://%s.local", hostname);
    return true;
}

void WiFiManager::loadConfig(bool forceFactory)
{

    auto loadIP = [&](const char *key, IPAddress fallback, IPAddress &target)
    {
        String val = _prefs.getString(key, fallback.toString());
        IPAddress tmp;
        target = tmp.fromString(val) ? tmp : fallback;
    };

    _prefs.begin("networkAP", false);

    if (forceFactory)
    {
        ESP_LOGW(TAG, "Factory reset → clearing preferences");

        _prefs.clear();
        _prefs.end();

        _netAccessPoint = _netFactory;
        saveConfig("networkAP", _netAccessPoint);

        ESP_LOGI(TAG, "AP Factory Config:");
        ESP_LOGI(TAG, "IP: %s", _netFactory.ip.toString().c_str());
        ESP_LOGI(TAG, "GW: %s", _netFactory.gateway.toString().c_str());
        ESP_LOGI(TAG, "SN: %s", _netFactory.subnet.toString().c_str());
        return;
    }

    // AP config
    if (!_prefs.isKey("ssid"))
    {
        _prefs.end();
        _netAccessPoint = _netFactory;
        saveAPConfig(_netAccessPoint);
        _prefs.begin("networkAP", false);
    }

    _netAccessPoint.ssid = _prefs.getString("ssid", _netFactory.ssid);
    _netAccessPoint.password = _prefs.getString("pw", _netFactory.password);

    loadIP("ip", _netFactory.ip, _netAccessPoint.ip);
    loadIP("gw", _netFactory.gateway, _netAccessPoint.gateway);
    loadIP("sn", _netFactory.subnet, _netAccessPoint.subnet);

    _prefs.end();

    ESP_LOGI(TAG, "AP Config:");
    ESP_LOGI(TAG, "IP: %s", _netAccessPoint.ip.toString().c_str());
    ESP_LOGI(TAG, "GW: %s", _netAccessPoint.gateway.toString().c_str());
    ESP_LOGI(TAG, "SN: %s", _netAccessPoint.subnet.toString().c_str());

    _prefs.begin("networkSTA", false);
    if (!_prefs.isKey("ssid"))
    {
        _prefs.end();

        saveSTAConfig(_netStation);
        _prefs.begin("networkSTA", false);
    }
    _netStation.ssid = _prefs.getString("ssid", _netStation.ssid);
    _netStation.password = _prefs.getString("pw", _netStation.password);

    _prefs.end();
}

void WiFiManager::saveAPConfig(const NetworkConfig &cfg)
{
    saveConfig("networkAP", cfg);
    _netAccessPoint = cfg;
}

void WiFiManager::saveSTAConfig(const NetworkConfig &cfg)
{
    saveConfig("networkSTA", cfg);
    _netStation = cfg;
}

void WiFiManager::saveConfig(const char *nameSpace, const NetworkConfig &cfg)
{
    _prefs.begin(nameSpace, false);

    _prefs.putString("ssid", cfg.ssid);
    _prefs.putString("pw", cfg.password);
    _prefs.putString("ip", cfg.ip.toString());
    _prefs.putString("gw", cfg.gateway.toString());
    _prefs.putString("sn", cfg.subnet.toString());

    _prefs.end();

    ESP_LOGI(TAG, "Config saved on Namespace %s", nameSpace);
}

bool WiFiManager::isFactoryReset()
{
    bool active = digitalRead(_resetPin) == LOW;

    if (active)
    {
        ESP_LOGW(TAG, "Factory reset pin active");
    }

    return active;
}

void WiFiManager::writeNetworkConfig(JsonDocument &doc)
{

    doc["ap_ssid"] = _netAccessPoint.ssid;
    doc["ap_password"] = "";
    doc["ap_ip"] = _netAccessPoint.ip.toString();
    doc["ap_gateway"] = _netAccessPoint.gateway.toString();
    doc["ap_subnet"] = _netAccessPoint.subnet.toString();

    doc["sta_ssid"] = _netStation.ssid;
    doc["sta_password"] = "";

    doc["sta_state"] = STAStateToString(_staState);
    doc["sta_status_code"] = WiFi.status();

    // ── IP info
    doc["sta_ip"] = WiFi.localIP().toString();
    doc["ap_ip_runtime"] = WiFi.softAPIP().toString();

    // ── optional: signal strength
    if (_staState == STAState::CONNECTED)
    {
        doc["rssi"] = WiFi.RSSI();
    }
}

void WiFiManager::checkStateChanges()
{

    uint16_t changeMask = 0;
    const uint32_t now = millis();

    if (_staState != _lastSTAState)
        changeMask |= WiFiManagerEvents::StateChangeBits::CHANGE_STA;

    if (changeMask)
        EventBus::instance()
            .publish(Event{EventType::NETWORK_STATE_CHANGE, changeMask,
                           now});

    _lastSTAState = _staState;
}

uint8_t WiFiManager::findBestChannel()
{
    ESP_LOGI(TAG, "Scanning WiFi networks...");

    int n = WiFi.scanNetworks();

    if (n <= 0)
    {
        ESP_LOGW(TAG, "No networks found, fallback to channel 6");
        return 6;
    }

    // Wir bewerten nur die Hauptkanäle
    int channelScore[12] = {0}; // index 1–11

    for (int i = 0; i < n; ++i)
    {
        int ch = WiFi.channel(i);
        int rssi = WiFi.RSSI(i);

        // Gewichtung: starke Signale zählen mehr
        int weight = map(rssi, -100, -30, 1, 10);

        if (ch >= 1 && ch <= 11)
        {
            channelScore[ch] += weight;
        }
    }

    WiFi.scanDelete();

    // Nur 1,6,11 vergleichen
    int candidates[] = {1, 6, 11};
    int bestChannel = 6;
    int bestScore = INT_MAX;

    for (int ch : candidates)
    {
        if (channelScore[ch] < bestScore)
        {
            bestScore = channelScore[ch];
            bestChannel = ch;
        }

        ESP_LOGI(TAG, "Channel %d score: %d", ch, channelScore[ch]);
    }

    ESP_LOGI(TAG, "Best channel: %d", bestChannel);

    return bestChannel;
}