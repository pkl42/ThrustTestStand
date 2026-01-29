
#include "comms/WebServerController.h"
#include <ArduinoJson.h>

void WebServerController::setupProtocolRoutes()
{

    // Protocol content
    server.on("/api/protocols/load", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  if (!request->hasParam("file"))
                  {
                      request->send(400, "application/json",
                                    "{\"error\":\"Missing file parameter\"}");
                      return;
                  }

                  String filePath = request->getParam("file")->value();
                  ESP_LOGI("WebServerController", "/api/protocols/load file: %s", filePath.c_str());

                  if (!filePath.startsWith("/protocols/"))
                  {
                      request->send(403, "application/json",
                                    "{\"error\":\"Invalid protocol path\"}");
                      return;
                  }

                  File f = LittleFS.open(filePath, "r");
                  if (!f)
                  {
                      request->send(404, "application/json",
                                    "{\"error\":\"Protocol not found\"}");
                      return;
                  }

                  DynamicJsonDocument doc(4096);
                  DeserializationError err = deserializeJson(doc, f);
                  f.close();

                  if (err)
                  {
                      request->send(500, "application/json",
                                    "{\"error\":\"Failed to parse protocol\"}");
                      return;
                  }

                  String out;
                  serializeJson(doc, out);
                  request->send(200, "application/json", out);
              });

    // List protocols
    server.on("/api/protocols", HTTP_GET, [this](AsyncWebServerRequest *request)
              {
                ESP_LOGI("WebServerController", "/api/protocols - HTTP_GET");
        StaticJsonDocument<128 + 80 * MAX_PROTOCOLS> doc;

        if (!_protocolManager->fillProtocolListJson(doc))
        {
            request->send(500, "application/json", "{\"error\":\"Failed to list protocols\"}");
            return;
        }

        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    // Upload protocol
    server.on("/api/protocols", HTTP_POST, [](AsyncWebServerRequest * /*req*/) {}, nullptr, [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
              {
                ESP_LOGI("WebServerController", "/api/protocols - HTTP_POST");
                  static String body;

                  if (index == 0) body = "";
                  body += String((char*)data).substring(0, len);

                  if (index + len != total) return;

                  String id, version, error;
                  if (!_protocolManager->storeProtocol(body, id, version, error))
                  {
                      request->send(400, "application/json", "{\"error\":\"" + error + "\"}");
                      return;
                  }

                  request->send(200, "application/json",
                                "{\"status\":\"ok\",\"id\":\"" + id + "\",\"version\":\"" + version + "\"}"); });

    // Delete protocol
    server.on("/api/protocols", HTTP_DELETE, [this](AsyncWebServerRequest *request)
              {
                 ESP_LOGI("WebServerController", "/api/protocols - HTTP_DELETE");
        if (!request->hasParam("id") || !request->hasParam("version"))
        {
            request->send(400, "application/json", "{\"error\":\"Missing id or version\"}");
            return;
        }

        const String id = request->getParam("id")->value();
        const String version = request->getParam("version")->value();

        if (!_protocolManager->deleteProtocol(id, version))
        {
            request->send(404, "application/json", "{\"error\":\"Protocol not found\"}");
            return;
        }

        request->send(200, "application/json", "{\"status\":\"deleted\"}"); });
}