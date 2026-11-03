/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include <ArduinoJson.h>

void WebServerController::sendCalibrationConfig(AsyncWebSocketClient *client)
{
    if (!_thrustStand)
        return;

    StaticJsonDocument<512> doc;

    doc["type"] = MsgType::CALIBRATION;
    ApiSerializer::writeCalibrationConfig(doc, *_thrustStand);

    sendJson(doc, client);
}

void WebServerController::setupCalibrationRoutes()
{
    //
    // ── GET /api/calibration/escProtocols
    //
    server.on("/api/calibration/escProtocols", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<256> doc;

                  ApiSerializer::writeESCDriverOptions(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });

    //
    // ── GET /api/calibration/rpmOptions
    //
    server.on("/api/calibration/rpmOptions", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<512> doc;

                  ApiSerializer::writeRpmOptions(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });
    //
    //
    // ── GET /api/calibration/
    //
    // server.on("/api/calibration", HTTP_GET,
    //           [this](AsyncWebServerRequest *request)
    //           {
    //               StaticJsonDocument<512> doc;

    //               _thrustStand->fillCalibrationJson(doc);

    //               AsyncResponseStream *response =
    //                   request->beginResponseStream("application/json");

    //               serializeJson(doc, *response);
    //               request->send(response);
    //           });
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

            // ---------- Temperature Calibration ----------
            if (doc.containsKey("temp"))
            {
                JsonObject tempObj = doc["temp"].as<JsonObject>();

                float coefficient = tempObj.containsKey("coefficient") ? tempObj["coefficient"] : 1.0f;
                float offset = tempObj.containsKey("offset") ? tempObj["offset"] : 0.0f;

                if (tempObj.containsKey("offset") || tempObj.containsKey("coefficient"))
                {
                    _thrustStand->setTemperatueCalibration(coefficient, offset);
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
                ESCConfig escConfig = _thrustStand->getESCDriverConfig();
                bool update_flag = false;

                if (motorObj.containsKey("pwm_min_us"))
                {
                    escConfig.minPulseUs = motorObj["pwm_min_us"].as<uint16_t>();
                    update_flag = true;
                }
                if (motorObj.containsKey("pwm_max_us"))
                {
                    escConfig.maxPulseUs = motorObj["pwm_max_us"].as<uint16_t>();
                    update_flag = true;
                }

                if (motorObj.containsKey("min_spin_percent"))
                {
                    escConfig.minSpinPercentage = motorObj["min_spin_percent"].as<uint8_t>();
                    update_flag = true;
                }

                if (update_flag)
                {
                    update_flag = _thrustStand->setESCDriverConfig(escConfig);
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
                    handled = _thrustStand->setESCDriver(static_cast<EscDriverType>(t));
                }
            }

            // ---------- RPM Configuration ----------
            if (doc.containsKey("rpm"))
            {
                JsonObject tempObj = doc["rpm"].as<JsonObject>();
                RpmUiConfig rpmUIConfig = _thrustStand->getRPMUICalibration();
                if (tempObj.containsKey("filterStrength"))
                    rpmUIConfig.filterLevel =
                        static_cast<RpmFilterLevel>(tempObj["filterStrength"].as<uint8_t>());

                if (tempObj.containsKey("updateRate"))
                    rpmUIConfig.responseMode =
                        static_cast<RpmResponseMode>(tempObj["updateRate"].as<uint8_t>());

                if (tempObj.containsKey("smoothing"))
                    rpmUIConfig.smoothing =
                        static_cast<RpmSmoothing>(tempObj["smoothing"].as<uint8_t>());

                if (tempObj.containsKey("pulsesPerRevolution"))
                    rpmUIConfig.pulsesPerRevolution =
                        tempObj["pulsesPerRevolution"].as<uint8_t>();

                if (tempObj.containsKey("filterStrength") ||
                    tempObj.containsKey("updateRate") ||
                    tempObj.containsKey("smoothing") ||
                    tempObj.containsKey("pulsesPerRevolution"))
                {

                    bool update_flag = _thrustStand->setRPMUICalibration(rpmUIConfig);
                    handled = true;
                    ESP_LOGI("WebServerController", "/api/calibration - _thrustStand->setRPMUICalibration %i", update_flag);
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
            sendCalibrationConfig();
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
                String("{\"currentSensitivity\":") + String(newSens, 6) + "}");
            sendCalibrationConfig(); });
}