/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEST_PROTOCOL_PARSER_H
#define TEST_PROTOCOL_PARSER_H

#include <ArduinoJson.h>
#include <Arduino.h>
#include "protocol/TestProtocol.h"

/**
 * @class TestProtocolParser
 * @brief JSON parser and validator for test protocols.
 *
 * Converts a JSON-based test protocol definition into an internal
 * TestProtocol structure. The parser performs structural validation
 * and reports detailed human-readable error messages on failure.
 *
 * The parser is designed to be:
 * - Stateless
 * - Deterministic
 * - Side-effect free on failure
 *
 * @section protocol_parser_responsibilities Responsibilities
 * - Validate protocol structure and required fields
 * - Reject unknown or unsupported keys
 * - Populate TestProtocol with validated values
 *
 * @section protocol_parser_thread_safety Thread / Execution Context Safety
 * - All methods must be called from non-ISR context
 * - This class is stateless and reentrant
 */
class TestProtocolParser
{
public:
    /**
     * @brief Parse a test protocol from a JSON document.
     *
     * On success, the output protocol structure is fully populated.
     * On failure, the output structure remains unchanged and a
     * descriptive error message is returned.
     *
     * @param doc      Parsed ArduinoJson document.
     * @param out      Output protocol structure.
     * @param outError Error description on failure.
     *
     * @return true if parsing and validation succeeded.
     */
    static bool parse(const JsonDocument &doc,
                      TestProtocol &out,
                      String &outError);

    /**
     * @brief Validate that no unknown keys are present in a JSON object.
     *
     * Checks the given JSON object against a whitelist of allowed keys
     * and reports the first unknown key encountered.
     *
     * @param obj         JSON object to validate.
     * @param allowedKeys Array of allowed key strings.
     * @param keyCount    Number of allowed keys.
     * @param outError    Error description on failure.
     *
     * @return true if all keys are allowed, false otherwise.
     */
    static bool checkUnknownKeys(const JsonObjectConst &obj,
                                 const char *const *allowedKeys,
                                 size_t keyCount,
                                 String &outError);
};

#endif // TEST_PROTOCOL_PARSER_H
