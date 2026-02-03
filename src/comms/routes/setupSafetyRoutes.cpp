/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include "core/SafetyLimits.h"
#include <ArduinoJson.h>

void WebServerController::setupSafetyRoutes()
{

    //
    // ── POST /api/safety/clear
    //
    server.on("/api/safety/clear", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
                ESP_LOGI("WebServerController", "/api/safety/clear - HTTP_POST");
    _thrustStand->clearSafetyTrip();
    request->send(200, "text/plain", "OK"); });

    //
    // ── GET /api/safety/battery-presets  --- GET battery preset settings
    //
    server.on("/api/safety/battery-presets", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  ESP_LOGI("WebServerController", "/api/safety/battery-presets - HTTP_GET");
                  StaticJsonDocument<768> doc;
                  _thrustStand->fillBatteryPresetJson(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });

    //
    // ── GET /api/safety  --- GET current safety settings
    //
    server.on("/api/safety", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  ESP_LOGI("WebServerController", "/api/safety - HTTP_GET");
                  StaticJsonDocument<256> doc;
                  _thrustStand->fillSafetyJson(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });

    //
    // ── POST /api/safety -- post limits
    //
    server.on(
        "/api/safety",
        HTTP_POST,
        [this](AsyncWebServerRequest *request)
        {
            // headers-only handler (unused)
        },
        nullptr,
        [this](AsyncWebServerRequest *request,
               uint8_t *data,
               size_t len,
               size_t index,
               size_t total)
        {
            ESP_LOGI("WebServerController", "/api/safety - HTTP_POST");
            StaticJsonDocument<512> doc;
            DeserializationError err = deserializeJson(doc, data, len);

            if (err)
            {
                request->send(400, "text/plain", "Invalid JSON");
                return;
            }

            if (!doc.containsKey("limits"))
            {
                request->send(400, "text/plain", "Missing limits object");
                return;
            }

            JsonObject limits = doc["limits"];

            // Apply limits (only if present)
            if (limits.containsKey("voltage_max_v"))
                _thrustStand->setVoltageLimitMaxV(
                    limits["voltage_max_v"], SafetyTripSource::CORE_LIMIT);

            if (limits.containsKey("voltage_min_v"))
                _thrustStand->setVoltageLimitMinV(
                    limits["voltage_min_v"], SafetyTripSource::CORE_LIMIT);

            if (limits.containsKey("thrust_gf"))
                _thrustStand->setThrustLimitGF(
                    limits["thrust_gf"], SafetyTripSource::CORE_LIMIT);

            if (limits.containsKey("current_a"))
                _thrustStand->setCurrentLimitA(
                    limits["current_a"], SafetyTripSource::CORE_LIMIT);

            if (limits.containsKey("throttle_percent"))
                _thrustStand->setThrottleLimitPercent(
                    limits["throttle_percent"], SafetyTripSource::CORE_LIMIT);

            if (limits.containsKey("battery_cells"))
            {

                uint8_t cells = limits["battery_cells"];
                BatteryPreset preset = cellsToPreset(cells);

                _thrustStand->setBatteryPreset(
                    preset,
                    SafetyTripSource::CORE_LIMIT);
            }
            StaticJsonDocument<64> resp;
            resp["status"] = "ok";

            String out;
            serializeJson(resp, out);
            request->send(200, "application/json", out);
        });
}