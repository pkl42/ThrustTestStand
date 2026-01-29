/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <Arduino.h>
#include <esp_log.h>
#include <pgmspace.h>
#include <freertos/semphr.h>
#include "core/Config.h"
#include "core/ThrustStand.h"
#include "protocol/TestProtocolExecutor.h"
#include "protocol/ProtocolManager.h"
#include "recorder/TestDataRecorder.h"
#include "comms/WebServerController.h"
#include "comms/WiFiAPController.h"

#define SERIAL_SPEED 115200

ThrustStand thrustStand;
TestDataRecorder dataRecorder(thrustStand);
ProtocolManager protocolManager;
TestProtocolExecutor testExecutor(thrustStand);

WiFiAPController wifiAP(AP_SSID, AP_PASSWORD);

// Global Instance
WebServerController *webServer;

SemaphoreHandle_t dataMutex;

// Forward Declaration der Tasks
void thrustStandTask(void *parameter);
void webTask(void *parameter);

void setup()
{
  Serial.begin(SERIAL_SPEED);
  delay(200);
  esp_log_level_set("*", ESP_LOG_INFO);
  ESP_LOGI("setup", "# %s %s | %s ", THRUSTSTAND_APP_NAME, THRUSTSTAND_APP_VERSION, THRUSTSTAND_BUILD_DATE);
  Serial.flush();

  // WiFi AP starten
  wifiAP.setAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  wifiAP.begin();

  // start Thrust Stand
  while (!thrustStand.init())
  {
    ESP_LOGE("setup", "thrust stand initializing failed!");
    delay(1000);
  };

  dataRecorder.begin();
  // initialize Test Executor
  testExecutor.begin();
  testExecutor.setRecorder(&dataRecorder);

  webServer = new WebServerController(&thrustStand, &testExecutor, &dataRecorder, &protocolManager);

  // Mutex für shared data
  dataMutex = xSemaphoreCreateMutex();
  if (!dataMutex)
  {
    ESP_LOGE("setup", "Failed to create Mutex!");
  }

  // Sensor/Motor-Task Core 0
  xTaskCreatePinnedToCore(
      thrustStandTask,
      "thrustStandTask",
      4096,
      NULL,
      2,
      NULL,
      0 // Core 0
  );

  // Webserver-Task Core 1
  xTaskCreatePinnedToCore(
      webTask,
      "WebTask",
      8192,
      NULL,
      1,
      NULL,
      1 // Core 1
  );
}

void loop()
{
}

// ================= Task Implementierungen =================

// Sensor & Motor Task (Core 0)
void thrustStandTask(void *parameter)
{
  const TickType_t loopDelay = pdMS_TO_TICKS(10); // 100 Hz

  while (true)
  {
    // 1️⃣ Update hardware (motor, sensors, ESC state machine)
    thrustStand.update();

    // 2️⃣ Run thrust test state machine (ONLY here!)
    testExecutor.update();
    dataRecorder.update();

    // 3️⃣ Protect shared data only if actually accessed
    if (dataMutex != nullptr)
    {
      if (xSemaphoreTake(dataMutex, 0) == pdTRUE)
      {
        // Example:
        // sharedData = thrustStand.getCurrentData();

        xSemaphoreGive(dataMutex);
      }
    }

    vTaskDelay(loopDelay);
  }
}

// Webserver Task (Core 1)
void webTask(void *parameter)
{
  // Webserver starten
  webServer->begin();

  // Task Loop
  while (true)
  {
    // AsyncWebServer erledigt alles andere
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
