/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef WEBSERVERCONTROLLER_H
#define WEBSERVERCONTROLLER_H

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

// Core
#include "core/ThrustStand.h"

// Protocol & test execution
#include "protocol/ProtocolManager.h"
#include "protocol/TestProtocolExecutor.h"

// Data recording / export
#include "recorder/TestDataRecorder.h"

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
     */
    WebServerController(ThrustStand *stand,
                        TestProtocolExecutor *executor,
                        TestDataRecorder *recorder,
                        ProtocolManager *protocolManager);

    /**
     * @brief Initialize filesystem, configure routes and start the web server.
     */
    void begin();

private:
    /**
     * @brief Asynchronous web server instance (port 80).
     */
    AsyncWebServer server{80};

    static const char *TAG; ///< Logging tag

    // ---- Domain references (non-owning) ----
    ThrustStand *_thrustStand;
    TestProtocolExecutor *_executor;
    TestDataRecorder *_recorder;
    ProtocolManager *_protocolManager;

    // ---- Route setup orchestration ----
    void setupStaticFiles();
    void setupApiRoutes();

    // ---- API route groups ----
    void setupLiveRoutes(); ///< /api/live/*
    void setupTestRoutes(); ///< /api/test/*

    void setupProtocolRoutes();    ///< /api/protocols/*
    void setupSystemRoutes();      ///< /api/system/*
    void setupSafetyRoutes();      ///< /api/safety/*
    void setupCalibrationRoutes(); ///< /api/calibration/*
    void setupDownloadRoutes();    ///< /api/download/*
};

#endif // WEBSERVERCONTROLLER_H
