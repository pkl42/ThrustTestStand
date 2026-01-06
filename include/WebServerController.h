#ifndef WEBSERVERCONTROLLER_H
#define WEBSERVERCONTROLLER_H

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include "ThrustStand.h"
#include "ThrustTestController.h"

/**
 * @brief Web interface controller for the thrust test stand.
 *
 * This class encapsulates an asynchronous HTTP web server running on the ESP32.
 * It exposes status information and control endpoints for the thrust stand and
 * the thrust test controller, and serves static or dynamically generated web pages.
 *
 * The server uses ESPAsyncWebServer and LittleFS for non-blocking operation
 * and file storage.
 */
class WebServerController
{
public:
    /**
     * @brief Construct a new WebServerController.
     *
     * @param stand Pointer to the ThrustStand instance providing sensor data.
     * @param controller Pointer to the ThrustTestController handling test logic.
     */
    WebServerController(ThrustStand *stand, ThrustTestController *controller);

    /**
     * @brief Initialize and start the web server.
     *
     * This function mounts the filesystem (if required), configures all HTTP
     * routes, and starts listening for incoming client connections.
     */
    void begin();

private:
    /**
     * @brief Asynchronous web server instance.
     *
     * Listens on port 80 and handles all HTTP requests non-blocking.
     */
    AsyncWebServer server{80};

    /**
     * @brief Pointer to the thrust stand hardware abstraction.
     */
    ThrustStand *thrustStand;

    /**
     * @brief Pointer to the thrust test controller.
     */
    ThrustTestController *testController;

    /**
     * @brief Configure all HTTP routes and request handlers.
     *
     * This method registers REST-style API endpoints for:
     *  - Live measurement data (JSON)
     *  - Calibration and configuration
     *  - Test control (start/stop)
     *  - Manual throttle control
     *  - Sensor taring
     *  - CSV test data download
     *
     * API routes are registered first, followed by static file handling
     * from LittleFS. All routes are handled asynchronously and must be
     * non-blocking.
     */
    void setupRoutes();

    /**
     * @brief Generate the main index page HTML.
     *
     * @return HTML content for the index page as a String.
     */
    String indexPage();
};

#endif // WEBSERVERCONTROLLER_H
