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
    ProtocolManager *protocolManager,
    WiFiManager *wifiManager)
    : _thrustStand(stand),
      _executor(executor),
      _recorder(recorder),
      _protocolManager(protocolManager),
      _wifiManager(wifiManager)
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

    setupWebSocket(); // before server.begin)

    setupApiRoutes();
    setupStaticFiles();

    server.begin();
    // subscribe event bus
    EventBus::instance().subscribe(
        [this](const Event &event)
        {
            handleEvent(event);
        });
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
    setupNetworkRoutes();
}

void WebServerController::setupWebSocket()
{

    _ws.onEvent([this](AsyncWebSocket *server,
                       AsyncWebSocketClient *client,
                       AwsEventType type,
                       void *arg,
                       uint8_t *data,
                       size_t len)
                {
        switch (type)
        {
        case WS_EVT_CONNECT:
            ESP_LOGI(TAG, "WS client connected: %u", client->id());
            sendInitialState(client);
            break;

        case WS_EVT_DISCONNECT:
            ESP_LOGI(TAG, "WS client disconnected: %u", client->id());
            break;

        case WS_EVT_DATA:
            // 🔥 (future) handle incoming commands
            ESP_LOGI(TAG, "WS data received");
            break;

        default:
            break;
        } });

    server.addHandler(&_ws); // 🔥 VERY IMPORTANT
    ESP_LOGI(TAG, "WebSocket handler added to Server");
}

void WebServerController::update()
{
    _ws.cleanupClients();
    sendLiveState();
    if (_executor->isRunning())
        sendTestState();
}

void WebServerController::sendJson(
    JsonDocument &doc,
    AsyncWebSocketClient *client)
{
    String msg;
    serializeJson(doc, msg);

    if (client)
        client->text(msg);
    else
        _ws.textAll(msg);
}

void WebServerController::sendInitialState(AsyncWebSocketClient *client)
{
    sendSystemState(client);
    sendSafetyConfig(client);
    sendProtocolList(client);
    // sendLiveState(client); is broadcasted by update()

    sendNetworkState(client);
    sendCalibrationConfig(client);
    sendTestConfiguration(client);
    sendTestState(client);
}

void WebServerController ::handleEvent(const Event &event)
{
    switch (event.type)
    {
    case EventType::SYSTEM_STATE_CHANGE:
        sendSystemState();
        break;
    case EventType::NETWORK_STATE_CHANGE:
        sendNetworkState();
        break;

    default:
        // optional: ignore unknown
        break;
    }
}