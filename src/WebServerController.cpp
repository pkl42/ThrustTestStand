#include "WebServerController.h"
#include "Version.h"

WebServerController::WebServerController(ThrustStand *stand, ThrustTestController *controller)
{
    thrustStand = stand;
    testController = controller;
}

#define FORMAT_IF_FAILED true

// Mount LITTLEFS
namespace
{
    bool fs_initialized = false; // Ensure SPIFFS.begin() runs only once

    void ensureFS()
    {
        if (!fs_initialized)
        {
            if (!LittleFS.begin(FORMAT_IF_FAILED)) // true: format if mount fails
            {
                ESP_LOGE("FS", "Failed to mount Filesystem");
            }
            else
            {
                ESP_LOGI("FS", "Filesystem mounted successfully");
                fs_initialized = true;
            }
        }
    }

} // namespace

void WebServerController::begin()
{
    ensureFS();
    //
    const char *path = "/index.html";
    if (!LittleFS.exists(path))
    {
        ESP_LOGE("WebServerController", "File does not exist: %s", path);
        return;
    }

    // DEBUG: Dateien auflisten
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file)
    {
        ESP_LOGI("WebServerController", "LittleFS file: %s  size %i", file.name(), file.size());

        file = root.openNextFile();
    }
    setupRoutes();
    server.begin();
    ESP_LOGI("WebServerController", "Webserver started");
}

void WebServerController::setupRoutes()
{
    // ---------- API ROUTES FIRST ----------
    server.on("/system", HTTP_GET, [this](AsyncWebServerRequest *request)
              {
    char buf[256];
    ThrustStandState s = thrustStand->getState();

    snprintf(buf, sizeof(buf),
        "{"
            "\"thrust\":%u,"
            "\"torque\":%u,"
            "\"thermocouple\":%u,"
            "\"rpm\":%u,"
            "\"current\":%u,"
             "\"voltage\":%u,"
            "\"motor\":%u,"
            "\"motor_mount\":%u"
        "}",
        (uint8_t)s.thrustSensor,
        (uint8_t)s.torqueSensor,
        (uint8_t)s.thermocoupleSensor,
        (uint8_t)s.rpmSensor,
        (uint8_t)s.currentSensor,
        (uint8_t)s.voltageSensor,
        (uint8_t)s.motor,
        (uint8_t)s.motorMountState
    );

    request->send(200, "application/json", buf); });

    // Live JSON Status
    server.on("/status", HTTP_GET, [this](AsyncWebServerRequest *request)
              {
        char buf[768];
        test_data_t d = thrustStand->getCurrentDataSet();
        TestStatus_t ts = testController->getStatus();
        test_data_accu_t c=thrustStand->getCumDataSet();

snprintf(buf, sizeof(buf),
        "{"
            "\"throttle\":%.1f,"
            "\"thrust\":%.2f,"
            "\"torque\":%.2f,"
            "\"torqueCell1\":%.2f,"
            "\"torqueCell2\":%.2f,"
            "\"voltage\":%.2f,"
            "\"current\":%.2f,"
             "\"power\":%.2f,"           
            "\"rpm\":%.0f,"
            "\"temperature\":%.2f,"
            "\"temperature_max\":%.2f,"
            "\"thrust_samples\":%u,"
            "\"torque_samples\":%u,"
            "\"test_running\":%u,"
            "\"test_step\":%u,"
            "\"test_steps_total\":%u,"
            "\"test_progress\":%.1f"
        "}",
        d.throttle,
        d.thrust,
        d.torque,
        d.torque_cell_1,
        d.torque_cell_2,
        d.voltage,
        d.current,
        d.power,
        d.rpm,
        d.temperature,
        d.temperature_max,
        c.samples.thrust,
        c.samples.torque,
        ts.running ? 1 : 0,
        ts.step,
        ts.total_steps,
        ts.progress
    );

        request->send(200, "application/json", buf); });

    // Calibration
    // GET config

    server.on("/config", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  response->print("{");

                  /* ---------- Version ---------- */
                  response->printf(
                      "\"version\": \"%s\",",
                      THRUSTSTAND_APP_VERSION);

                  /* ---------- Thrust ---------- */
                  response->printf(
                      "\"thrust_cal\": %.4f,",
                      thrustStand->getThrustCalFactor());

                  /* -------- MaxThrust --------- */
                  response->printf(
                      "\"thrust_max\": %.0f",
                      thrustStand->getMaxThrottlePercent());

                  /* ---------- Torque ---------- */
                  auto torqueCal =
                      thrustStand->getTorqueCalibration();

                  response->printf(
                      ",\"torque_cal1\": %.4f"
                      ",\"torque_cal2\": %.4f"
                      ",\"torque_dist\": %.4f",
                      torqueCal.cal1,
                      torqueCal.cal2,
                      torqueCal.distanceMM);
                  /* ---------- Current Sensitivity ---------- */
                  float currentSensitivity =
                      thrustStand->getCurrentSensitivity();
                  response->printf(
                      ",\"current_sensitivity\": %.4f", currentSensitivity);

                  /* ---------- Motor PWM Range ---------- */
                  uint16_t minPulseUs =
                      thrustStand->getMinPulseUs();
                  response->printf(
                      ",\"minPulseUs\": %i", minPulseUs);
                  uint16_t maxPulseUs =
                      thrustStand->getMaxPulseUs();
                  response->printf(
                      ",\"maxPulseUs\": %i", maxPulseUs);

                  response->print("}");

                  request->send(response);
              });

    server.on("/api/test/metadata", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  const TestMetadata_t &md = testController->getMetadata();

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  response->print("{");
                  response->printf(
                      "\"motorType\": \"%s\",",
                      md.motorType.c_str());
                  response->printf(
                      "\"escType\": \"%s\",",
                      md.escType.c_str());
                  response->printf(
                      "\"propellerType\": \"%s\"",
                      md.propellerType.c_str());
                  response->print("}");

                  request->send(response);
              });

    // Config Section
    server.on("/config", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  bool handled = false;

                  /* ---------- Thrust Calibration ---------- */
                  if (request->hasParam("thrust_cal", true))
                  {
                      float cal = request->getParam("thrust_cal", true)->value().toFloat();
                      thrustStand->setThrustCalFactor(cal);
                      handled = true;
                  }

                  /* ---------- Max Thrust  ----------- */
                  if (request->hasParam("thrust_max", true))
                  {
                      float max = request->getParam("thrust_max", true)->value().toFloat();
                      ESP_LOGI("setupRoutes",
                               "setMaxThrottlePercent: max=%.0f",
                               max);
                      thrustStand->setMaxThrottlePercent(max);
                      handled = true;
                  }

                  /* ---------- Torque Calibration ---------- */
                  bool hasTorqueParam = false;

                  float cal1 = 0.0f;
                  float cal2 = 0.0f;
                  float dist = 0.0f;

                  if (request->hasParam("torqueCal1", true))
                  {
                      cal1 = request->getParam("torqueCal1", true)->value().toFloat();
                      hasTorqueParam = true;
                  }

                  if (request->hasParam("torqueCal2", true))
                  {
                      cal2 = request->getParam("torqueCal2", true)->value().toFloat();
                      hasTorqueParam = true;
                  }

                  if (request->hasParam("torqueDistance", true))
                  {
                      dist = request->getParam("torqueDistance", true)->value().toFloat();
                      hasTorqueParam = true;
                  }

                  if (hasTorqueParam)
                  {
                      ESP_LOGI("setupRoutes",
                               "TorqueCal: cal1=%.3f cal2=%.3f dist=%.3f",
                               cal1, cal2, dist);
                      thrustStand->setTorqueCalibration(cal1, cal2, dist);
                      handled = true;
                  }

                  /* ---------- Current Calibration ---------- */
                  if (request->hasParam("current_sensitivity", true))
                  {
                      float current_sensitivity = request->getParam("current_sensitivity", true)->value().toFloat();
                      thrustStand->setCurrentSensitivity(current_sensitivity);
                      ESP_LOGI("setupRoutes",
                               "setCurrentSensitivity:%.4f",
                               current_sensitivity);
                      handled = true;
                  }

                  /* ---------- Motor PWM Range ---------- */

                  if (request->hasParam("minPulseUs", true))
                  {
                      uint16_t minPulseUs = request->getParam("minPulseUs", true)->value().toInt();
                      if (request->hasParam("maxPulseUs", true))
                      {
                          uint16_t maxPulseUs = request->getParam("maxPulseUs", true)->value().toInt();
                          bool updateFlag = thrustStand->setPulseRangeUs(minPulseUs, maxPulseUs);
                          if (updateFlag)
                              handled = true;
                      }
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
    // Tare
    server.on("/tare", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
                ESP_LOGI("WebServerController", "tareSensors");
        thrustStand->tareSensors();
        request->send(200, "text/plain", "OK"); });

    // CSV Download
    server.on("/downloadCSV", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        if (LittleFS.exists("/last_test.csv")) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS,
                 "/last_test.csv", "text/csv");
            response->addHeader(
                "Content-Disposition",
                "attachment; filename=\"thrustTest.csv\""
            );
             request->send(response);
        } else {
            request->send(404, "text/plain", "CSV file not found");
        } });

    // Set Throttle
    server.on("/setThrottle", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
        if (request->hasParam("throttle", true)) {
            float val = request->getParam("throttle", true)->value().toFloat();
            ESP_LOGI("WebServerController", "setThrottle %f",val);
            thrustStand->setThrottle(val);
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Missing param");
        } });

    // Start / Stop Test
    server.on("/api/test/metadata", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  ESP_LOGI("WebServerController", "setTestMetadata");

                  if (testController->isTestRunning())
                  {
                      request->send(409, "text/plain", "Test running");
                      return;
                  }

                  if (!request->hasArg("motorType") || !request->hasArg("escType") || !request->hasArg("propellerType"))
                  {
                      request->send(400, "text/plain", "Missing parameters");
                      return;
                  }

                  bool ok = testController->setMetadata(
                      request->arg("motorType"),
                      request->arg("escType"),
                      request->arg("propellerType"));

                  if (!ok)
                  {
                      request->send(409, "text/plain", "Cannot set metadata");
                      return;
                  }

                  request->send(200, "text/plain", "OK");
              });

    server.on("/api/test/start", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  ESP_LOGI("WebServerController", "startTest");

                  if (!testController->startTest())
                  {
                      request->send(409, "text/plain", "Cannot start test");
                      return;
                  }

                  request->send(200, "text/plain", "OK");
              });

    server.on("/api/test/stop", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
                ESP_LOGI("WebServerController", "stopTest");
        testController->stopTest();
        request->send(200, "text/plain", "OK"); });

    // favicon (optional)
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(204); });

    // ---------- STATIC FILES LAST ----------
    server.serveStatic("/", LittleFS, "/")
        .setDefaultFile("index.html")
        .setCacheControl("no-cache")
        .setTryGzipFirst(false); // until index.html.gz is additional generated in parallel to index.html
}
