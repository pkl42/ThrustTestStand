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
    server.on("/api/calibration", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  bool handled = false;

                  /* ---------- Thrust Calibration ---------- */
                  if (request->hasParam("thrust", true))
                  {
                      float cal = request->getParam("thrust", true)->value().toFloat();
                      _thrustStand->setThrustCalFactor(cal);
                      handled = true;
                  }

                  /* ---------- Torque Calibration ---------- */
                  bool hasTorqueParam = false;

                  float cal1 = 0.0f;
                  float cal2 = 0.0f;
                  float dist = 0.0f;

                  if (request->hasParam("torque.cal1", true))
                  {
                      cal1 = request->getParam("torque.cal1", true)->value().toFloat();
                      hasTorqueParam = true;
                  }

                  if (request->hasParam("torque.cal2", true))
                  {
                      cal2 = request->getParam("torque.cal2", true)->value().toFloat();
                      hasTorqueParam = true;
                  }

                  if (request->hasParam("torque.distance_mm", true))
                  {
                      dist = request->getParam("torque.distance_mm", true)->value().toFloat();
                      hasTorqueParam = true;
                  }

                  if (hasTorqueParam)
                  {
                      ESP_LOGI("setupRoutes",
                               "TorqueCal: torque.cal1=%.3f torque.cal2=%.3f torque.distance_mm=%.3f",
                               cal1, cal2, dist);
                      _thrustStand->setTorqueCalibration(cal1, cal2, dist);
                      handled = true;
                  }

                  /* ---------- Current Calibration ---------- */
                  if (request->hasParam("current.sensitivity", true))
                  {
                      float current_sensitivity = request->getParam("current.sensitivity", true)->value().toFloat();
                      _thrustStand->setCurrentSensitivity(current_sensitivity);
                      ESP_LOGI("setupRoutes",
                               "setCurrentSensitivity:%.4f",
                               current_sensitivity);
                      handled = true;
                  }

                  /* ---------- Volate Calibration ---------- */
                  if (request->hasParam("voltage_calibration", true))
                  {
                      float voltage_calibration = request->getParam("voltage_calibration", true)->value().toFloat();
                      _thrustStand->setVoltageCalibrationFactor(voltage_calibration);
                      ESP_LOGI("setupRoutes",
                               "setVoltageCalibrationFactor:%.2f",
                               voltage_calibration);
                      handled = true;
                  }

                  /* ---------- Motor PWM Range ---------- */

                  if (request->hasParam("minPulseUs", true))
                  {
                      uint16_t minPulseUs = request->getParam("minPulseUs", true)->value().toInt();
                      if (request->hasParam("maxPulseUs", true))
                      {
                          uint16_t maxPulseUs = request->getParam("maxPulseUs", true)->value().toInt();
                          bool updateFlag = _thrustStand->setPulseRangeUs(minPulseUs, maxPulseUs);
                          if (updateFlag)
                              handled = true;
                      }
                  }

                  if (request->hasParam("manual_idle_disarm_timeout_s", true))
                  {
                      uint32_t manual_idle_disarm_timeout_s = request->getParam("manual_idle_disarm_timeout_s", true)->value().toInt();
                      _thrustStand->setAutoDisarmTimeOut(manual_idle_disarm_timeout_s);
                      ESP_LOGI("setupRoutes",
                               "setAutoDisarmTimeOut:%u",
                               manual_idle_disarm_timeout_s);
                      handled = true;
                  }

                  /* ---------- Response ---------- */
                  if (handled)
                  {
                      request->send(200, "text/plain", "OK");
                  }
                  else
                  {
                      request->send(400, "text/plain", "Missing or invalid parameters");
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