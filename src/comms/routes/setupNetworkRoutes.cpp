/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include <ArduinoJson.h>

bool parseIP(JsonObject obj, const char *key, IPAddress &target)
{
    if (!obj.containsKey(key))
        return false;

    String value = obj[key] | "";
    if (value.isEmpty())
        return false;

    IPAddress tmp;
    if (tmp.fromString(value))
    {
        target = tmp;
        return true;
    }

    return false;
}

void WebServerController::sendNetworkState(AsyncWebSocketClient *client)
{
    StaticJsonDocument<256> doc; // same size as used in fillLiveSnapshot

    doc["type"] = MsgType::NETWORK;
    _wifiManager->writeNetworkConfig(doc);
    sendJson(doc, client);
}

void WebServerController::setupNetworkRoutes()
{
    //
    // ── GET /api/network
    //
    // server.on("/api/network", HTTP_GET,
    //           [this](AsyncWebServerRequest *request)
    //           {
    //               StaticJsonDocument<256> doc;

    //               _wifiManager->fillNetworkJson(doc);

    //               AsyncResponseStream *response =
    //                   request->beginResponseStream("application/json");

    //               serializeJson(doc, *response);
    //               request->send(response);
    //           });

    //
    // ── POST /api/network/reconnect
    //
    server.on("/api/network/reconnect", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
                ESP_LOGI("WebServerController", "/api/network/reconnect - HTTP_POST");
                _wifiManager->reconnectSTA();

    request->send(200, "application/json", "{\"status\":\"reconnecting\"}"); });

    //
    // ── POST /api/network
    //
    server.on(
        "/api/network",
        HTTP_POST,
        [](AsyncWebServerRequest *request)
        {
            // No-op headers-only handler
        },
        nullptr,
        [this](AsyncWebServerRequest *request,
               uint8_t *data,
               size_t len,
               size_t index,
               size_t total)
        {
            StaticJsonDocument<512> doc;
            DeserializationError err = deserializeJson(doc, data, len);

            if (err)
            {
                request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                return;
            }

            bool handled = false;

            // ---------- Network ----------
            if (doc.containsKey("network"))
            {
                NetworkConfig netAccessPoint = _wifiManager->getAPConfig();

                JsonObject networkObj = doc["network"].as<JsonObject>();
                if (networkObj.containsKey("ap_ssid"))
                {
                    String newSsid = networkObj["ap_ssid"].as<String>();
                    if (newSsid.length() > 0)
                    {
                        netAccessPoint.ssid = newSsid;
                    }
                }

                if (networkObj.containsKey("ap_password"))
                {
                    String newPass = networkObj["ap_password"].as<String>();
                    if (newPass.length() > 0)
                    {
                        netAccessPoint.password = newPass;
                        ESP_LOGI("WebServerController", "New AP-Password will be set.");
                    }
                    else
                    {
                        ESP_LOGI("WebServerController", "AP-Password-Field empty. Keep old password.");
                    }
                }
                parseIP(networkObj, "ap_ip", netAccessPoint.ip);
                parseIP(networkObj, "ap_gateway", netAccessPoint.gateway);
                parseIP(networkObj, "ap_subnet", netAccessPoint.subnet);

                ESP_LOGI("WebServerController", "/api/network - HTTP_POST - saveAPConfig");
                _wifiManager->saveAPConfig(netAccessPoint);

                NetworkConfig netStation = _wifiManager->getSTAPConfig();
                netStation.ssid = networkObj["sta_ssid"] | netStation.ssid;

                if (networkObj.containsKey("sta_password"))
                {
                    String newPass = networkObj["sta_password"].as<String>();
                    if (newPass.length() > 0)
                    {
                        netStation.password = newPass;
                        ESP_LOGI("WebServerController", "New STA-Password will be set.");
                    }
                    else
                    {
                        ESP_LOGI("WebServerController", "STA-Password-Field empty. Keep old password.");
                    }
                }
                ESP_LOGI("WebServerController", "/api/network - HTTP_POST - saveSTAConfig");
                _wifiManager->saveSTAConfig(netStation);
                handled = true;
            }

            // ---------- Response ----------
            if (handled)
            {
                StaticJsonDocument<16> resp;
                resp["status"] = "ok";
                String out;
                serializeJson(resp, out);
                request->send(200, "application/json", out);
            }
            else
            {
                request->send(400, "application/json", "{\"error\":\"Missing or invalid parameters\"}");
            }
        });
}
