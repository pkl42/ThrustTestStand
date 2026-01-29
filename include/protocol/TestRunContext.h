/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TEST_RUN_CONTEXT_H
#define TEST_RUN_CONTEXT_H

#include <Arduino.h>

/**
 * @brief Metadata describing a test run.
 *
 * Includes motor, ESC, propeller, and protocol identifiers.
 * Stored with recorded data and used in CSV export / web UI.
 */
struct TestRunContext
{
    String motorType;       ///< Motor identifier or description
    String escType;         ///< ESC identifier or description
    String propellerType;   ///< Propeller identifier or description
    String protocolID;      ///< ID of the protocol being executed
                            // String protocolID;      ///< ID of the protocol being executed
    String protocolVersion; ///< Optional version of the protocol

    char csvFormat[3] = ".,";
};

#endif // TEST_RUN_CONTEXT_H
