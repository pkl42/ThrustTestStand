/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include <ArduinoJson.h>

void WebServerController::setupCalibrationRoutes()
{
    //
    // ── GET /api/calibration/escProtocols
    //
    server.on("/api/calibration/escProtocols", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<256> doc;

                  _thrustStand->fillESCDriverJson(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });
    //
    // ── GET /api/calibration/calibrate_current
    //
    server.on("/api/calibration", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<512> doc;

                  _thrustStand->fillCalibrationJson(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });
    //
    // ── POST /api/calibration
    //
    server.on(
        "/api/calibration",
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

            // ---------- Thrust Calibration ----------
            if (doc.containsKey("thrust"))
            {
                JsonObject thrustObj = doc["thrust"].as<JsonObject>();
                if (thrustObj.containsKey("cal"))
                {
                    float cal = thrustObj["cal"];
                    _thrustStand->setThrustCalFactor(cal);
                    handled = true;
                }
            }

            // ---------- Torque Calibration ----------
            if (doc.containsKey("torque"))
            {
                JsonObject torqueObj = doc["torque"].as<JsonObject>();

                float cal1 = torqueObj.containsKey("cal1") ? torqueObj["cal1"] : 0.0f;
                float cal2 = torqueObj.containsKey("cal2") ? torqueObj["cal2"] : 0.0f;
                float dist = torqueObj.containsKey("distance_mm") ? torqueObj["distance_mm"] : 0.0f;

                if (torqueObj.containsKey("cal1") || torqueObj.containsKey("cal2") || torqueObj.containsKey("distance_mm"))
                {
                    _thrustStand->setTorqueCalibration(cal1, cal2, dist);
                    handled = true;
                }
            }

            // ---------- Current Calibration ----------
            if (doc.containsKey("current"))
            {
                JsonObject currentObj = doc["current"].as<JsonObject>();
                if (currentObj.containsKey("sensitivity"))
                {
                    float s = currentObj["sensitivity"];
                    _thrustStand->setCurrentSensitivity(s);
                    handled = true;
                }
            }

            // ---------- Voltage Calibration ----------
            if (doc.containsKey("voltage"))
            {
                JsonObject voltageObj = doc["voltage"].as<JsonObject>();
                if (voltageObj.containsKey("calibration"))
                {
                    float v = voltageObj["calibration"];
                    _thrustStand->setVoltageCalibrationFactor(v);
                    handled = true;
                }
            }

            // ---------- Motor PWM Range ----------
            if (doc.containsKey("motor"))
            {
                JsonObject motorObj = doc["motor"].as<JsonObject>();
                if (motorObj.containsKey("pwm_min_us") && motorObj.containsKey("pwm_max_us"))
                {
                    uint16_t minUs = motorObj["pwm_min_us"];
                    uint16_t maxUs = motorObj["pwm_max_us"];
                    _thrustStand->setPulseRangeUs(minUs, maxUs);
                    handled = true;
                }
                if (motorObj.containsKey("disarm_timeout_s"))
                {
                    uint32_t t = motorObj["disarm_timeout_s"];
                    _thrustStand->setAutoDisarmTimeOut(t);
                    handled = true;
                }
                if (motorObj.containsKey("esc_driver_type"))
                {
                    uint8_t t = motorObj["esc_driver_type"];
                    handled = _thrustStand->switchDriver(static_cast<EscDriverType>(t));
                }
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

    //
    // ── POST /api/calibration/calibrate_current
    //
    server.on("/api/calibration/calibrate_current", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
            if (!request->hasParam("actual_current", true)) {
                request->send(400, "text/plain", "Missing actual_current");
                return;
            }

            float actual = request->getParam("actual_current", true)->value().toFloat();
            float measured = NAN;

            if (request->hasParam("measured_current", true)) {
                measured = request->getParam("measured_current", true)->value().toFloat();
            }

            _thrustStand->autoCalibrateCurrentSensor(actual, measured);

            float newSens = _thrustStand->getCurrentSensitivity();

            request->send(200, "application/json",
                String("{\"currentSensitivity\":") + String(newSens, 6) + "}"); });
}