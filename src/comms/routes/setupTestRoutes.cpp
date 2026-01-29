/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "comms/WebServerController.h"
#include "protocol/TestProtocolParser.h"
#include <ArduinoJson.h>

void WebServerController::setupTestRoutes()
{
    //
    // ── GET /api/test/config
    //
    server.on("/api/test/config", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<512> doc;
                  _executor->fillConfigJson(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });
    //
    // ── GET /api/test
    //
    server.on("/api/test", HTTP_GET,
              [this](AsyncWebServerRequest *request)
              {
                  StaticJsonDocument<512> doc;
                  _executor->fillStatusJson(doc);

                  AsyncResponseStream *response =
                      request->beginResponseStream("application/json");

                  serializeJson(doc, *response);
                  request->send(response);
              });
    //
    // ── PUT /api/test/config
    //
    server.on(
        "/api/test/config",
        HTTP_PUT,
        [this](AsyncWebServerRequest *request) {},
        nullptr,
        [this](AsyncWebServerRequest *request,
               uint8_t *data,
               size_t len,
               size_t index,
               size_t total)
        {
            static String body;

            if (index == 0)
                body.clear();

            body += String((char *)data).substring(0, len);

            if (index + len != total)
                return;

            if (_executor->isRunning())
            {
                request->send(409, "application/json",
                              "{\"error\":\"test_running\"}");
                return;
            }

            StaticJsonDocument<512> doc;
            DeserializationError err = deserializeJson(doc, body);
            if (err)
            {
                request->send(400, "application/json",
                              "{\"error\":\"invalid_json\"}");
                return;
            }
            TestRunContext ctx = _executor->getRunContext();

            const char *csvFmt = nullptr;
            if (doc.containsKey("csvFormat"))
                csvFmt = doc["csvFormat"];

            bool ok = _executor->setRunContext(
                doc["motorType"] | ctx.motorType,
                doc["escType"] | ctx.escType,
                doc["propellerType"] | ctx.propellerType,
                doc["protocolID"] | ctx.protocolID,
                doc["protocolVersion"] | ctx.protocolVersion,
                csvFmt);

            if (!ok)
            {
                request->send(409, "application/json",
                              "{\"error\":\"cannot_set_context\"}");
                return;
            }

            request->send(200, "application/json", "{\"status\":\"ok\"}");
        });
    //
    // ── POST /api/test/start
    //
    server.on("/api/test/start", HTTP_POST,
              [this](AsyncWebServerRequest *request)
              {
                  if (_executor->isRunning())
                  {
                      request->send(409, "application/json",
                                    "{\"error\":\"test_running\"}");
                      return;
                  }

                  TestRunContext ctx = _executor->getRunContext();

                  /* ---------- Load protocol JSON ---------- */
                  String json;
                  if (!_protocolManager->loadProtocol(ctx.protocolID, ctx.protocolVersion, json))
                  {
                      request->send(404, "application/json",
                                    "{\"error\":\"protocol_not_found\"}");
                      return;
                  }

                  /* ---------- Parse JSON ---------- */
                  StaticJsonDocument<4096> doc;

                  DeserializationError err = deserializeJson(doc, json);

                  if (err)
                  {
                      ESP_LOGI("WebServerController", "/api/test/start JSON parse failed: %s", err.c_str());
                      request->send(400, "application/json",
                                    "{\"error\":\"invalid_protocol_json\"}");
                      return;
                  }

                  TestProtocol proto;
                  String parseError;
                  if (!TestProtocolParser::parse(doc, proto, parseError))
                  {
                      ESP_LOGI("WebServerController", "JSON parsed, memory usage: %u bytes", doc.memoryUsage());
                      request->send(400, "application/json",
                                    "{\"error\":\"protocol_parse_failed\",\"detail\":\"" +
                                        parseError + "\"}");
                      return;
                  }

                  /* ---------- Start execution ---------- */
                  auto errStart = _executor->start(proto);
                  if (errStart != StartError::Ok)
                  {
                      ESP_LOGI("WebServerController", "_executor->start errored: %s", String(static_cast<int>(errStart)).c_str());
                      request->send(409, "application/json",
                                    "{\"ok\":false,\"error\":" + String(static_cast<int>(errStart)) + "}");
                  }
                  else
                  {
                      //   ESP_LOGI("WebServerController",
                      //            "Test started: %s (%u steps)",
                      //            proto.name,
                      //            proto.steps.size());

                      request->send(200, "application/json",
                                    "{\"ok\":true,\"status\":\"started\"}");
                  }
              });

    // ── POST /api/test/stop
    server.on("/api/test/stop", HTTP_POST, [this](AsyncWebServerRequest *request)
              {
                ESP_LOGI("WebServerController", "stopTest");
        _executor->stop(ExecState::Aborted);
        request->send(200, "text/plain", "OK"); });

    // calculate simple checksum (optional)
    // if (!doc.containsKey("checksum")) {
    //     uint32_t sum = 0;
    //     for (size_t i = 0; i < json.length(); ++i) sum += json[i];
    //     doc["checksum"] = String(sum, HEX);
    // }

    // AsyncResponseStream *response = request->beginResponseStream("application/json");
    // serializeJson(doc, *response);
    // request->send(response); });
}
