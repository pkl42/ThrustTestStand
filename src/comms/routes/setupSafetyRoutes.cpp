/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
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
    server.on("/api/safety", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  ESP_LOGI("WebServerController", "/api/safety - HTTP_POST");
                  bool handled = false;

                  if (request->hasParam("limits.voltage_max_v", true))
                  {
                      float voltage_max_v = request->getParam("limits.voltage_max_v", true)->value().toFloat();
                      _thrustStand->setVoltageLimitMaxV(voltage_max_v, SafetyTripSource::CORE_LIMIT);
                      handled = true;
                  }

                  if (request->hasParam("limits.voltage_min_v", true))
                  {
                      float voltage_min_v = request->getParam("limits.voltage_min_v", true)->value().toFloat();
                      _thrustStand->setVoltageLimitMinV(voltage_min_v, SafetyTripSource::CORE_LIMIT);
                      handled = true;
                  }

                  if (request->hasParam("limits.thrust_gf", true))
                  {
                      float thrustGF = request->getParam("limits.thrust_gf", true)->value().toFloat();

                      _thrustStand->setThrustLimitGF(thrustGF, SafetyTripSource::CORE_LIMIT);
                      handled = true;
                  }

                  if (request->hasParam("limits.current_a", true))
                  {
                      float currentA = request->getParam("limits.current_a", true)->value().toFloat();

                      _thrustStand->setCurrentLimitA(currentA, SafetyTripSource::CORE_LIMIT);
                      handled = true;
                  }

                  if (request->hasParam("limits.throttle_percent", true))
                  {
                      float percent = request->getParam("limits.throttle_percent", true)->value().toFloat();

                      _thrustStand->setThrottleLimitPercent(percent, SafetyTripSource::CORE_LIMIT);
                      handled = true;
                  }

                  if (handled)
                  {
                      request->send(200, "text/plain", "OK");
                  }
                  else
                  {
                      request->send(400, "text/plain", "Missing or invalid parameters");
                  }
              });
}