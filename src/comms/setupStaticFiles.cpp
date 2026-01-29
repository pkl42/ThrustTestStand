/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include <ArduinoJson.h>

void WebServerController::setupStaticFiles()
{
    server.on("/favicon.ico", HTTP_GET,
              [](AsyncWebServerRequest *request)
              { request->send(204); });

    // ---------- STATIC FILES LAST ----------
    server.serveStatic("/", LittleFS, "/")
        .setDefaultFile("index.html")
        .setCacheControl("no-cache")
        .setTryGzipFirst(false); // until index.html.gz is additional generated in parallel to index.html
}
