/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SENSOR_BASE_H
#define SENSOR_BASE_H
#include "SensorState.h"

class BaseSensor
{
public:
    virtual ~BaseSensor() = default;

    // Core sensor action
    virtual bool update() = 0;

    // Lifecycle

    SensorState getState() const;
    bool isReady() const;
    bool isInitialized() const;

    // Data validity
    bool hasValidData() const;

    // Diagnostics
    uint32_t getUpdateCount() const;
    uint32_t getErrorCount() const;

protected:
    BaseSensor();

    // State control for derived classes
    void setState(SensorState state);
    void setDataValid(bool valid);
    void incrementError();

protected:
    SensorState _state;
    bool _dataValid;
    uint32_t _updateCount;
    uint32_t _errorCount;
};

#endif