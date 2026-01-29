/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include <ArduinoJson.h>

void WebServerController::setupDownloadRoutes()
{

    //
    // ── GET /api/export/csv/mean -- CSV Download average/mean
    //
    server.on("/api/export/csv/mean", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        if (LittleFS.exists("/last_test_mean.csv")) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS,
                 "/last_test_mean.csv", "text/csv");
            response->addHeader(
                "Content-Disposition",
                "attachment; filename=\"thrustTest_mean.csv\""
            );
             request->send(response);
        } else {
            request->send(404, "text/plain", "CSV file not found");
        } });

    //
    // ── GET /api/export/csv/statistics -- CSV Download full statistics
    //
    server.on("/api/export/csv/statistics", HTTP_GET,
              [](AsyncWebServerRequest *request)
              {
                  if (LittleFS.exists("/last_test_stats.csv"))
                  {
                      AsyncWebServerResponse *response =
                          request->beginResponse(
                              LittleFS,
                              "/last_test_stats.csv",
                              "text/csv");

                      response->addHeader(
                          "Content-Disposition",
                          "attachment; filename=\"thrustTest_stats.csv\"");

                      request->send(response);
                  }
                  else
                  {
                      request->send(404, "text/plain", "Statistics CSV file not found");
                  }
              });
}
