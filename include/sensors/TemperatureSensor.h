/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "core/Config.h"
#include "BaseTemperatureSensor.h"

/**
 * @brief Compile-time selection of the temperature sensor implementation.
 *
 * The concrete sensor driver is selected via @ref TEMPERATURE_SENSOR_TYPE
 * defined in Config.h.
 *
 * This mechanism allows:
 * - Switching hardware without changing application code
 * - Zero runtime overhead (compile-time binding)
 *
 * Supported sensor types:
 * - TEMP_SENSOR_MAX31855 → @ref TempMax31855Sensor
 * - TEMP_SENSOR_MLX90614 → @ref TempMax90614Sensor
 *
 * If no valid type is selected, a dummy implementation is used.
 *
 * @note
 * The selected type is aliased as @ref SelectedTemperatureSensor.
 */

#if TEMPERATURE_SENSOR_TYPE == TEMP_SENSOR_MAX31855
#include "sensors/TempMAX31855Sensor.h"
using SelectedTemperatureSensor = TempMax31855Sensor;

#elif TEMPERATURE_SENSOR_TYPE == TEMP_SENSOR_MLX90614
#include "sensors/TempMAX90614Sensor.h"
using SelectedTemperatureSensor = TempMax90614Sensor;

#else
/**
 * @class DummyTemperatureSensor
 * @brief Fallback sensor implementation when no valid sensor is configured.
 *
 * This implementation:
 * - Always fails initialization
 * - Never produces valid data
 *
 * Used to ensure the system compiles even if no sensor is selected.
 *
 * @warning
 * Should not be used in production configurations.
 */
class DummyTemperatureSensor : public BaseTemperatureSensor
{
public:
    bool begin(...) { return false; }
    bool update() override { return false; }
};
using SelectedTemperatureSensor = DummyTemperatureSensor;

#endif

// Wrapper-Class

/**
 * @class TemperatureSensor
 * @brief Unified wrapper for the selected temperature sensor implementation.
 *
 * Provides a consistent interface independent of the underlying hardware.
 * The actual implementation is selected at compile time via
 * @ref SelectedTemperatureSensor.
 *
 * @section wrapper_purpose Purpose
 *
 * - Abstracts hardware-specific initialization parameters
 * - Provides a stable API for higher-level components
 * - Encapsulates compile-time configuration logic
 *
 *
 * @section wrapper_behavior Behavior
 *
 * - Inherits all functionality from the selected sensor implementation
 * - Only overrides begin() to inject board-specific configuration
 * - All other methods (update, getTemperature, etc.) are forwarded
 *
 *
 * @section wrapper_selection Selection Mechanism
 *
 * The underlying implementation depends on:
 * - TEMPERATURE_SENSOR_TYPE (Config.h)
 *
 * Example mappings:
 * - MAX31855 → SPI-based thermocouple sensor
 * - MLX90614 → I2C infrared temperature sensor
 *
 *
 * @section wrapper_design Design Notes
 *
 * - No virtual dispatch overhead beyond BaseSensor
 * - No runtime branching (fully compile-time resolved)
 * - Keeps application code independent of hardware details
 *
 *
 * @note
 * This class should be used throughout the system instead of directly
 * instantiating hardware-specific sensor classes.
 */
class TemperatureSensor : public SelectedTemperatureSensor
{
public:
    /**
     * @brief Initialize the selected temperature sensor.
     *
     * Calls the appropriate begin() implementation with
     * hardware-specific pin configuration.
     *
     * @return true if initialization succeeded, false otherwise
     *
     * @note
     * - Pin configuration is resolved at compile time
     * - Behavior depends on selected sensor type
     */
    bool begin()
    {
#if TEMPERATURE_SENSOR_TYPE == TEMP_SENSOR_MAX31855
        return BaseTemperatureSensor::begin(MAX31855_CLK_PIN, MAX31855_CS_PIN, MAX31855_SO_PIN);
#elif TEMPERATURE_SENSOR_TYPE == TEMP_SENSOR_MLX90614
        return SelectedTemperatureSensor::begin(MLX90614_SDA_PIN, MLX90614_SCL_PIN);
#else
        return SelectedTemperatureSensor::begin(); // dummy
#endif
    }
};

#endif // TEMPERATURE_SENSOR_H