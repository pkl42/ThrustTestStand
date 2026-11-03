/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include <ArduinoJson.h>

void WebServerController::sendSystemState(AsyncWebSocketClient *client)
{
    StaticJsonDocument<512> doc; // same size as used in fillLiveSnapshot

    doc["type"] = MsgType::SYSTEM;
    ApiSerializer::writeSystemState(doc, *_thrustStand);

    sendJson(doc, client);
}

void WebServerController::setupSystemRoutes()
{

    //
    // ── GET /api/system
    //
    // server.on("/api/system", HTTP_GET,
    //           [this](AsyncWebServerRequest *request)
    //           {
    //               StaticJsonDocument<256> doc;

    //               _thrustStand->fillSystemStateJson(doc);

    //               AsyncResponseStream *response =
    //                   request->beginResponseStream("application/json");

    //               serializeJson(doc, *response);
    //               request->send(response);
    //           });

    //
    // ── POST /api/system/initialize  Initialize sensors & system
    //
    server.on("/api/system/initialize", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  ESP_LOGI("WebServerController", "/api/system/initialize - HTTP_POST");
                  StaticJsonDocument<256> doc;

                  bool success = _thrustStand->init_sensors(true); // only sensors
                  doc["success"] = success;
                  doc["message"] = success ? "Sensors initialized successfully" : "Sensor initialization failed";

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
                  sendSystemState();
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
                  sendSystemState();
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
                  sendSystemState();
              });

    //
    // ── POST /api/system/mode  --- change operating mode
    //
    server.on("/api/system/mode", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
                  // headers-only handler (unused)
              },
              nullptr, [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
              {
                 ESP_LOGI("WebServerController", "/api/system/mode - HTTP_POST");
                  StaticJsonDocument<256> doc;

                  DeserializationError err = deserializeJson(doc, data, len);

                              if (err)
            {
                request->send(400, "text/plain", "Invalid JSON");
                return;
            }

            if (!doc.containsKey("system"))
            {
                request->send(400, "text/plain", "Missing system object");
                return;
            }

            JsonObject system = doc["system"];

            bool success = false;
            if (system.containsKey("mode"))
            {

                  if (strcmp(system["mode"], "MODE_TEST") == 0)
                  {
                      _thrustStand->exitEscPassthrough();
                      success = true;
                  }
                  else if (strcmp(system["mode"], "MODE_ESC_PASSTHROUGH") == 0)
                  {
                      success = _thrustStand->enterEscPassthrough();
                  }

                  
                  doc["mode"] = systemModeToString(_thrustStand->getSystemMode()); // add this helper
                }
                doc["success"] = success;

                AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                serializeJson(doc, *response);
                request->send(response);
                sendSystemState(); });
}
