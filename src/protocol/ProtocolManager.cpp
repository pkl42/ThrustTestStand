/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "protocol/ProtocolManager.h"
#include <ArduinoJson.h>

const char *ProtocolManager::TAG = "ProtocolManager";

bool ProtocolManager::begin()
{
    if (!LittleFS.begin(true))
        return false;

    return ensureDirExists();
}

bool ProtocolManager::ensureDirExists() const
{
    if (LittleFS.exists(PROTOCOL_DIR))
        return true;

    return LittleFS.mkdir(PROTOCOL_DIR);
}

String ProtocolManager::makeFilename(const String &id,
                                     const String &version) const
{
    int maxIdLen = 30 - 5 - 2 - version.length(); // 5 = ".json", 2 = "_v"
    String safeId = id;
    if (safeId.length() > maxIdLen)
        safeId = safeId.substring(0, maxIdLen);

    return String(PROTOCOL_DIR) + "/" + safeId + "_v" + version + ".json";
}

bool ProtocolManager::exists(const String &id,
                             const String &version) const
{
    return LittleFS.exists(makeFilename(id, version));
}

bool ProtocolManager::storeProtocol(const String &json,
                                    String &outId,
                                    String &outVersion,
                                    String &outError)
{

    if (countProtocols() >= MAX_PROTOCOLS)
    {
        outError = "Protocol storage full";
        return false;
    }

    StaticJsonDocument<2048> doc;
    DeserializationError err = deserializeJson(doc, json);

    if (err)
    {
        outError = "Invalid JSON";
        return false;
    }

    if (!doc.containsKey("id") || !doc.containsKey("version"))
    {
        outError = "Missing id or version";
        return false;
    }

    outId = doc["id"].as<String>();
    outVersion = doc["version"].as<String>();

    if (outId.isEmpty() || outVersion.isEmpty())
    {
        outError = "Empty id or version";
        return false;
    }

    const String path = makeFilename(outId, outVersion);

    if (LittleFS.exists(path))
    {
        outError = "Protocol already exists";
        return false;
    }

    File f = LittleFS.open(path, "w");
    if (!f)
    {
        outError = "Failed to create file";
        return false;
    }

    f.print(json);
    f.close();

    return true;
}

bool ProtocolManager::deleteProtocol(const String &id,
                                     const String &version)
{
    const String path = makeFilename(id, version);

    if (!LittleFS.exists(path))
        return false;

    return LittleFS.remove(path);
}

bool ProtocolManager::loadProtocol(const String &id,
                                   const String &version,
                                   String &outJson) const
{
    const String path = makeFilename(id, version);

    if (!LittleFS.exists(path))
        return false;

    File f = LittleFS.open(path, "r");
    if (!f)
        return false;

    outJson = f.readString();
    f.close();

    return true;
}

bool ProtocolManager::fillProtocolListJson(JsonDocument &doc) const
{
    JsonArray arr = doc.to<JsonArray>();

    File dir = LittleFS.open(PROTOCOL_DIR, "r");
    if (!dir || !dir.isDirectory())
        return false;

    ESP_LOGI(TAG, "Scanning protocols directory...");

    while (true)
    {
        File f = dir.openNextFile();
        if (!f)
            break;

        if (f.isDirectory())
            continue;

        String name = f.name();
        ESP_LOGI(TAG, "Found entry: %s", name.c_str());

        if (!name.endsWith(".json"))
            continue;

        StaticJsonDocument<2048> docJson;
        docJson.clear();
        DeserializationError err = deserializeJson(docJson, f);
        f.close();

        if (err)
        {
            ESP_LOGE(TAG,
                     "Failed to parse %s: %s",
                     name.c_str(),
                     err.c_str());
            continue;
        }

        JsonObject obj = arr.createNestedObject();
        if (obj.isNull())
        {
            ESP_LOGE(TAG, "JSON capacity exhausted while adding protocol");
            return false;
        }

        ESP_LOGI(TAG,
                 "Parsed %s → id=%s version=%s",
                 name.c_str(),
                 docJson["id"] | "(missing)",
                 docJson["version"] | "(missing)");

        obj["id"] = docJson["id"].as<String>();
        obj["version"] = docJson["version"].as<String>();

        obj["file"] = String(PROTOCOL_DIR) + name;
    }

    return true;
}

uint8_t ProtocolManager::countProtocols() const
{
    uint8_t count = 0;

    File dir = LittleFS.open(PROTOCOL_DIR);
    if (!dir || !dir.isDirectory())
        return 0;

    File f = dir.openNextFile();
    while (f)
    {
        if (String(f.name()).endsWith(".json"))
            count++;

        f = dir.openNextFile();
    }
    return count;
}
