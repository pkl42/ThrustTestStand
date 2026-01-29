/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include <ArduinoJson.h>

void WebServerController::setupLiveRoutes()
{
    //
    // ── GET /api/live/snapshot -- live data
    //

    server.on("/api/live/snapshot", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<512> doc;

                  _thrustStand->fillLiveSnapshot(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });

    //
    // ── POST /api/live/setThrottle
    //
    server.on("/api/live/setThrottle", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
        if (request->hasParam("throttle", true)) {
            float val = request->getParam("throttle", true)->value().toFloat();
            ESP_LOGI("WebServerController", "setThrottle %f",val);
            _thrustStand->setThrottle(val);
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Missing param");
        } });

    //
    // ── POST /api/live/tare
    //
    server.on("/api/live/tare", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
    bool success = _thrustStand->tareSensors();
    ESP_LOGI("WebServerController", "tareSensors -> %s", success ? "OK" : "FAILED");

    if (success)
    {
        request->send(200, "text/plain", "OK");
    }
    else
    {
        request->send(500, "text/plain", "TARE_FAILED");
    } });
}