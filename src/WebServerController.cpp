#include "WebServerController.h"
#include "Config.h"

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
        test_data_accu_t c=thrustStand->getAccuDataSet();
        ThrustStandSafety s = thrustStand->getStandSafety();

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
             "\"thrust_ratio\":%.2f,"   
            "\"rpm\":%.0f,"
            "\"temperature\":%.2f,"
            "\"thrust_samples\":%u,"
            "\"torque_samples\":%u,"
            "\"test_running\":%u,"
            "\"test_step\":%u,"
            "\"test_steps_total\":%u,"
            "\"test_progress\":%.1f,"
            "\"safety_tripped\":%s,"
            "\"safety_trip_value\":%.2f,"
            "\"safety_trip_reason\":\"%s\""
        "}",
        d.throttle,
        d.thrust,
        d.torque,
        d.torque_cell_1,
        d.torque_cell_2,
        d.voltage,
        d.current,
        d.voltage * d.current,
        d.thrust/(d.voltage * d.current),
        d.rpm,
        d.temperature,
        c.thrust.mean,
        c.torque.mean,
        ts.running ? 1 : 0,
        ts.step,
        ts.total_steps,
        ts.progress,
        s.state.tripped ? "true" : "false",
        s.state.tripValue,
        thrustStand->safetyTripReasonToString(s.state.reason)
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

                  /* ----------Voltage ---------- */
                  float voltageCalibration =
                      thrustStand->getVoltageCalibrationFactor();
                  response->printf(
                      ",\"voltage_calibration\": %.4f", voltageCalibration);

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
                  const CsvFormat_t &csv = testController->getCsvFormat();

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
                      "\"propellerType\": \"%s\",",
                      md.propellerType.c_str());
                  response->printf(
                      "\"csvFormat\":\"%s\"",
                      csv.format);
                  response->print("}");

                  request->send(response);
              });

    // Safety

    server.on("/api/safety", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  const ThrustStandSafety &safety = thrustStand->getStandSafety();
                  const ThrustStandLimits &limits = safety.limits;
                  const ThrustStandSafetyState &state = safety.state;

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  response->print("{");

                  /* ---------- Limits ---------- */
                  response->printf(
                      "\"throttle_limit_percent\":%.2f,",
                      limits.maxThrottlePercent);

                  response->printf(
                      "\"current_limit_a\":%.2f,",
                      limits.maxCurrentA);

                  response->printf(
                      "\"voltage_limit_v\":%.2f,",
                      limits.maxVoltageV);

                  response->printf(
                      "\"thrust_limit_gf\":%.2f,",
                      limits.maxThrustGF);

                  /* ---------- Safety State ---------- */
                  response->printf(
                      "\"tripped\":%s,",
                      state.tripped ? "true" : "false");

                  response->printf(
                      "\"trip_value\":%.2f,",
                      state.tripValue);

                  response->printf(
                      "\"trip_reason\":\"%s\"",
                      thrustStand->safetyTripReasonToString(state.reason));

                  response->print("}");

                  request->send(response);
              });

    // Safety clear
    server.on("/api/safety/clear", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
    thrustStand->clearSafetyTrip();
    request->send(200, "text/plain", "OK"); });

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
                      thrustStand->setThrottleLimitPercent(max);

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

                  /* ---------- Volate Calibration ---------- */
                  if (request->hasParam("voltage_calibration", true))
                  {
                      float voltage_calibration = request->getParam("voltage_calibration", true)->value().toFloat();
                      thrustStand->setVoltageCalibrationFactor(voltage_calibration);
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

    server.on("/calibrate_current", HTTP_POST, [this](AsyncWebServerRequest *request)
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

    thrustStand->autoCalibrateCurrentSensor(actual, measured);

    float newSens = thrustStand->getCurrentSensitivity();

    request->send(200, "application/json",
        String("{\"currentSensitivity\":") + String(newSens, 6) + "}"); });

    // Tare
    server.on("/tare", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
    bool success = thrustStand->tareSensors();
    ESP_LOGI("WebServerController", "tareSensors -> %s", success ? "OK" : "FAILED");

    if (success)
    {
        request->send(200, "text/plain", "OK");
    }
    else
    {
        request->send(500, "text/plain", "TARE_FAILED");
    } });

    // CSV Download
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

                  if (request->hasParam("csvFormat", true))
                  {
                      testController->setCsvFormat(
                          request->getParam("csvFormat", true)->value().c_str());
                  }

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
