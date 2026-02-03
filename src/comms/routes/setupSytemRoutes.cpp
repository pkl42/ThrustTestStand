/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include <ArduinoJson.h>

void WebServerController::setupSystemRoutes()
{

    //
    // ── GET /api/system
    //
    server.on("/api/system", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<256> doc;

                  _thrustStand->fillSystemStateJson(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });

    //
    // ── POST /api/system/initialize  Initialize sensors & system
    //
    server.on("/api/system/initialize", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<256> doc;

                  bool success = _thrustStand->init_sensors(true); // only sensors
                  doc["success"] = success;
                  doc["message"] = success ? "Sensors initialized successfully" : "Sensor initialization failed";

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });

    //
    // ── POST /api/system/arm  --- Arm the moto
    //
    server.on("/api/system/arm", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<256> doc;

                  if (!_thrustStand->isSafeToArm())
                  {
                      doc["success"] = false;
                      doc["message"] = "Cannot arm: E-Stop active or cage open";
                  }
                  else
                  {
                      bool armed = _thrustStand->armMotor(); // new method
                      doc["success"] = armed;
                      doc["message"] = armed ? "Motor armed successfully" : "Motor failed to arm";
                  }

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });

    //
    // ── POST /api/system/disarm  --- Disarm the motor
    //
    server.on("/api/system/disarm", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<256> doc;

                  bool disarmed = _thrustStand->disarmMotor(); // new method
                  doc["success"] = disarmed;
                  doc["message"] = disarmed ? "Motor disarmed successfully" : "Motor failed to disarm";

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });
}
