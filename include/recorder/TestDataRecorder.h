/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TEST_DATA_RECORDER_H
#define TEST_DATA_RECORDER_H

#include <stdint.h>
#include <LittleFS.h>
#include <limits>
#include "recorder/SamplingWindow.h"
#include "core/ThrustStand.h"
#include "protocol/TestRunContext.h"
#include "protocol/TestProtocol.h"
#include "protocol/TestProtocolExecutor.h"

struct ProtocolExecStatus;
class ThrustStand;

static constexpr uint32_t MAX_RECORD_STEPS = 96;
static constexpr uint16_t INVALID_REC_INDEX = std::numeric_limits<uint16_t>::max();

/**
 * @brief CSV formatting configuration.
 *
 * Defines formatting characters used during CSV export.
 */
typedef struct
{
    char format[3] = ".,";
} CsvFormat_t;

/**
 * @class TestDataRecorder
 * @brief Records thrust stand measurement data and exports CSV files.
 *
 * The TestDataRecorder collects time-series and step-based measurement
 * data from the ThrustStand during a test run. Recorded data is stored
 * in RAM and can later be exported to CSV files using LittleFS.
 *
 * The recorder supports:
 * - Triggered and continuous sampling
 * - Fixed-size in-memory buffering
 * - Protocol step association
 * - Statistical post-processing
 *
 * @section recorder_lifecycle Recorder Lifecycle
 *
 * Typical usage:
 * @code
 * recorder.begin();
 * recorder.reset();
 * recorder.trigger(measurementConfig);
 * recorder.start();
 * ...
 * recorder.update();
 * ...
 * recorder.stop();
 * recorder.exportMeanCsv(...);
 * @endcode
 *
 * @section recorder_thread_safety Thread / Execution Context Safety
 * - All public methods must be called from non-ISR context.
 * - update() is expected to be called from the main loop().
 */
class TestDataRecorder
{
public:
    /**
     * @brief Construct a TestDataRecorder.
     *
     * @param stand Reference to the ThrustStand providing measurement data.
     */
    explicit TestDataRecorder(ThrustStand &stand);

    /**
     * @brief Initialize the recorder.
     *
     * Prepares internal buffers and filesystem access.
     *
     * @return true on successful initialization.
     */
    bool begin();

    /**
     * @brief Configure a measurement trigger.
     *
     * Arms the recorder to start sampling when the provided
     * measurement configuration conditions are met.
     *
     * @param mc Measurement configuration defining trigger conditions.
     */
    void trigger(const MeasurementConfig &mc);

    /**
     * @brief Start recording.
     *
     * Begins data acquisition according to the active measurement
     * configuration.
     */
    void start();

    /**
     * @brief Stop recording.
     *
     * Ends data acquisition and finalizes the current sampling window.
     */
    void stop();

    /**
     * @brief Reset the recorder state.
     *
     * Clears all previously recorded data and resets timestamps.
     *
     * @param ts_offset Timestamp offset in milliseconds.
     */
    void reset(uint32_t ts_offset = 0);

    /**
     * @brief Periodic update function.
     *
     * Must be called regularly from the main loop() when
     * time-series recording is active.
     */
    void update();

    /**
     * @brief Check whether recording is currently active.
     *
     * @return true if recording is in progress.
     */
    bool isRecording() const { return _recording; }

    /**
     * @brief Check whether the internal buffer is full.
     *
     * @return true if no additional samples can be recorded.
     */
    bool isFull() const
    {
        return _sampleIndex != INVALID_REC_INDEX &&
               _sampleIndex >= (MAX_RECORD_STEPS - 1);
    }

    /**
     * @brief Export mean values as CSV.
     *
     * Writes averaged measurement values per protocol step
     * to a CSV file.
     *
     * @param path        Filesystem path to write to.
     * @param testContext Test run metadata.
     * @param testResult  Protocol execution result and statistics.
     *
     * @return true on successful export.
     */
    bool exportMeanCsv(const char *path,
                       const TestRunContext &testContext,
                       const ProtocolExecStatus &testResult);

    /**
     * @brief Export full statistics as CSV.
     *
     * Writes detailed statistical data (min/max/mean/stddev)
     * per protocol step to a CSV file.
     *
     * @param path        Filesystem path to write to.
     * @param testContext Test run metadata.
     * @param testResult  Protocol execution result and statistics.
     *
     * @return true on successful export.
     */
    bool exportStatisticsCsv(const char *path,
                             const TestRunContext &testContext,
                             const ProtocolExecStatus &testResult);

private:
    static const char *TAG; ///< Logging tag

    ThrustStand &_stand; ///< Reference to the thrust stand data source

    bool _recording = false; ///< Recording active flag
    bool _sampling = false;  ///< Sampling window active flag

    uint16_t _sampleIndex = 0;        ///< Current sample index
    MeasurementConfig _measureConfig; ///< Active measurement configuration

    /**
     * @brief Recorded measurement data.
     *
     * Fixed-size buffer storing accumulated statistics per
     * protocol step.
     */
    test_data_accu_t _data[MAX_RECORD_STEPS];

    uint32_t _lastSampleMs = 0;     ///< Timestamp of last sample
    uint32_t _nextIntervalMs = 0;   ///< Next sampling deadline
    uint32_t _sampleIntervalMs = 0; ///< Sampling interval (ms)
    uint32_t _ts_offset = 0;        ///< Timestamp offset (ms)

    SamplingWindow _window; ///< Active sampling window

    /**
     * @brief Begin a new sampling window.
     */
    void beginSampling();

    /**
     * @brief End the current sampling window.
     */
    void endSampling();

    /**
     * @brief Write CSV metadata header.
     *
     * @param f           Open file handle.
     * @param testContext Test run metadata.
     * @param testResult  Protocol execution result.
     */
    void writeCsvMetadata(File &f,
                          const TestRunContext &testContext,
                          const ProtocolExecStatus &testResult);

    /**
     * @brief Print a floating-point value to CSV.
     *
     * @param f          Open file handle.
     * @param value      Value to print.
     * @param precision  Number of decimal places.
     * @param decimalSep Decimal separator character.
     */
    static void printFloat(File &f,
                           float value,
                           int precision,
                           char decimalSep);

    /**
     * @brief Print sensor statistics as CSV fields.
     *
     * @param f          Open file handle.
     * @param s          Sensor statistics.
     * @param decimals   Number of decimal places.
     * @param decimalSep Decimal separator character.
     * @param fieldSep   CSV field separator.
     */
    void printStats(File &f,
                    const sensor_stats_t &s,
                    uint8_t decimals,
                    char decimalSep,
                    char fieldSep);
};

#endif // TEST_DATA_RECORDER_H
