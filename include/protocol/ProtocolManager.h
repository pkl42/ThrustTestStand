/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PROTOCOL_MANAGER_H
#define PROTOCOL_MANAGER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

static constexpr uint8_t MAX_PROTOCOLS = 10;

/**
 * @class ProtocolManager
 * @brief Manages persistent storage of test protocols.
 *
 * The ProtocolManager is responsible for storing, loading,
 * listing, and deleting test protocol definitions in JSON form.
 *
 * Protocols are stored on the filesystem (LittleFS) and are
 * uniquely identified by their ID and version.
 *
 * @section protocol_manager_constraints Constraints
 * - Overwriting existing protocols is not allowed
 * - The maximum number of stored protocols is limited
 * - All operations are filesystem-backed
 *
 * @section protocol_manager_thread_safety Thread / Execution Context Safety
 * - All public methods must be called from non-ISR context
 */
class ProtocolManager
{
public:
    /**
     * @brief Initialize the protocol manager.
     *
     * Mounts the filesystem and prepares the protocol storage directory.
     *
     * @return true on successful initialization.
     */
    bool begin();

    /**
     * @brief Store a new protocol definition.
     *
     * Stores the provided protocol JSON on the filesystem.
     * Existing protocols with the same ID and version are rejected.
     *
     * @param json       Raw protocol JSON.
     * @param outId      Extracted protocol ID.
     * @param outVersion Extracted protocol version.
     * @param outError   Error description on failure.
     *
     * @return true if the protocol was stored successfully.
     */
    bool storeProtocol(const String &json,
                       String &outId,
                       String &outVersion,
                       String &outError);

    /**
     * @brief Delete a stored protocol.
     *
     * @param id      Protocol ID.
     * @param version Protocol version.
     *
     * @return true if the protocol was deleted.
     */
    bool deleteProtocol(const String &id,
                        const String &version);

    /**
     * @brief Check whether a protocol exists.
     *
     * @param id      Protocol ID.
     * @param version Protocol version.
     *
     * @return true if the protocol exists.
     */
    bool exists(const String &id,
                const String &version) const;

    /**
     * @brief Load a stored protocol JSON.
     *
     * @param id       Protocol ID.
     * @param version  Protocol version.
     * @param outJson  Loaded protocol JSON.
     *
     * @return true if the protocol was found and loaded.
     */
    bool loadProtocol(const String &id,
                      const String &version,
                      String &outJson) const;

    /**
     * @brief Fill JSON with the list of stored protocols.
     *
     * Appends protocol metadata (ID, version, etc.) to the
     * provided JSON document.
     *
     * @param doc ArduinoJson document to fill.
     * @return true on success.
     */
    bool fillProtocolListJson(JsonDocument &doc) const;

    /**
     * @brief Fill JSON for a single stored protocol.
     *
     * Loads and inserts the full protocol definition into
     * the provided JSON document.
     *
     * @param id       Protocol ID.
     * @param version  Protocol version.
     * @param doc      JSON document to fill.
     *
     * @return true if the protocol was found.
     */
    bool fillProtocolJson(const String &id,
                          const String &version,
                          JsonDocument &doc) const;

private:
    static const char *TAG; ///< Logging tag

    static constexpr const char *PROTOCOL_DIR = "/protocols/"; ///< Storage directory

    /**
     * @brief Count the number of stored protocols.
     *
     * @return Number of stored protocol files.
     */
    uint8_t countProtocols() const;

    /**
     * @brief Generate filesystem filename for a protocol.
     *
     * @param id      Protocol ID.
     * @param version Protocol version.
     *
     * @return Filesystem path.
     */
    String makeFilename(const String &id,
                        const String &version) const;

    /**
     * @brief Ensure protocol storage directory exists.
     *
     * @return true if directory exists or was created successfully.
     */
    bool ensureDirExists() const;
};

#endif // PROTOCOL_MANAGER_H
