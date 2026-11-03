/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef WEBSERVERCONTROLLER_H
#define WEBSERVERCONTROLLER_H

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include "WifiManager.h"

// Core
#include "core/ThrustStand.h"
#include "comms/APISerializer.h"

// Protocol & test execution
#include "protocol/ProtocolManager.h"
#include "protocol/TestProtocolExecutor.h"

// Data recording / export
#include "recorder/TestDataRecorder.h"

#include "event/EventBus.h"

namespace MsgType
{
    static constexpr const char *LIVE = "live";
    static constexpr const char *TEST_STATE = "test_status";
    static constexpr const char *TEST_CONFIG = "test_config";

    static constexpr const char *SYSTEM = "system";
    static constexpr const char *SAFETY = "safety_settings";
    static constexpr const char *CALIBRATION = "calibration";
    static constexpr const char *NETWORK = "network";
    static constexpr const char *PROTOCOL_LIST = "protocol_list";

}

/**
 * @brief Web interface controller for the thrust test stand.
 *
 * This class owns and configures the asynchronous HTTP server.
 * It exposes REST-style JSON APIs and serves static UI files
 * from LittleFS.
 *
 * Responsibilities:
 *  - API routing and request handling
 *  - JSON serialization
 *  - Delegation to domain objects (stand, executor, recorder)
 *
 * It does NOT:
 *  - Execute test logic
 *  - Perform measurements
 *  - Store protocol files directly
 */
class WebServerController
{
public:
    /**
     * @brief Construct a new WebServerController.
     *
     * @param stand Pointer to the thrust stand hardware abstraction
     * @param executor Pointer to the test protocol executor
     * @param recorder Pointer to the test data recorder
     * @param protocolManager Pointer to the protocol manager
     * @param wifiManager Pointer to the wifi manager
     */
    WebServerController(ThrustStand *stand,
                        TestProtocolExecutor *executor,
                        TestDataRecorder *recorder,
                        ProtocolManager *protocolManager,
                        WiFiManager *wifiManager);

    /**
     * @brief Initialize filesystem, configure routes and start the web server.
     */
    void begin();

    void update();

private:
    /**
     * @brief Asynchronous web server instance (port 80).
     */
    AsyncWebServer server{80};

    AsyncWebSocket _ws{"/ws"};

    void setupWebSocket();
    void sendJson(
        JsonDocument &doc,
        AsyncWebSocketClient *client);

    void sendInitialState(AsyncWebSocketClient *client);

    // ---- Domain references (non-owning) ----
    ThrustStand *_thrustStand;
    TestProtocolExecutor *_executor;
    TestDataRecorder *_recorder;
    ProtocolManager *_protocolManager;
    WiFiManager *_wifiManager;

    // Event Handler - used for push messages

    void handleEvent(const Event &event);

    // ---- Route setup orchestration ----
    void setupStaticFiles();
    void setupApiRoutes();

    // ---- API route groups and Push routines ----
    void setupLiveRoutes(); ///< /api/live/*
    void sendLiveState(AsyncWebSocketClient *client = nullptr);

    void setupTestRoutes(); ///< /api/test/*
    void sendTestConfiguration(AsyncWebSocketClient *client = nullptr);
    void sendTestState(AsyncWebSocketClient *client = nullptr);

    void setupProtocolRoutes(); ///< /api/protocols/*
    void sendProtocolList(AsyncWebSocketClient *client = nullptr);

    void setupSystemRoutes(); ///< /api/system/*
    void sendSystemState(AsyncWebSocketClient *client = nullptr);

    void setupSafetyRoutes(); ///< /api/safety/*
    void sendSafetyConfig(AsyncWebSocketClient *client = nullptr);

    void setupCalibrationRoutes(); ///< /api/calibration/*
    void sendCalibrationConfig(AsyncWebSocketClient *client = nullptr);

    void setupDownloadRoutes(); ///< /api/download/*

    void setupNetworkRoutes(); ///< /api/network/*
    void sendNetworkState(AsyncWebSocketClient *client = nullptr);
};

#endif // WEBSERVERCONTROLLER_H
