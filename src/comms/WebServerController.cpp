/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include "core/Config.h"

static const char *TAG = "WebServerController";

#define FORMAT_IF_FAILED true

namespace
{
    bool fs_initialized = false;

    void ensureFS()
    {
        if (!fs_initialized)
        {
            if (!LittleFS.begin(FORMAT_IF_FAILED))
            {
                ESP_LOGE("FS", "Failed to mount LittleFS");
            }
            else
            {
                ESP_LOGI("FS", "LittleFS mounted");
                fs_initialized = true;
            }
        }
    }
}

WebServerController::WebServerController(
    ThrustStand *stand,
    TestProtocolExecutor *executor,
    TestDataRecorder *recorder,
    ProtocolManager *protocolManager)
    : _thrustStand(stand),
      _executor(executor),
      _recorder(recorder),
      _protocolManager(protocolManager)
{
}

void WebServerController::begin()
{
    ensureFS();

    if (!LittleFS.exists("/index.html"))
    {
        ESP_LOGE("Web", "index.html missing");
        return;
    }

    setupApiRoutes();
    setupStaticFiles();

    server.begin();
    ESP_LOGI("Web", "Webserver started");
}

void WebServerController::setupApiRoutes()
{
    setupLiveRoutes();
    setupSystemRoutes();
    setupSafetyRoutes();
    setupTestRoutes();
    setupProtocolRoutes();
    setupCalibrationRoutes();
    setupDownloadRoutes();
}
