#ifndef THRUST_TEST_CONTROLLER_H
#define THRUST_TEST_CONTROLLER_H

#include <stdint.h>
#include <LittleFS.h>
// #include "ThrustStand.h"

class ThrustStand;

/**
 * @brief Status of an ongoing thrust test.
 *
 * Tracks the current test step, total number of steps, and
 * overall progress in percentage.
 */
typedef struct
{
    bool running;         ///< Test currently active
    uint16_t step;        ///< Current step index (0-based)
    uint16_t total_steps; ///< Total number of steps in the test
    float progress;       ///< Progress percentage (0.0 – 100.0 %)
} TestStatus_t;

/**
 * @brief Metadata describing the test configuration.
 */
typedef struct
{
    String motorType;     ///< Motor identifier or description
    String escType;       ///< ESC identifier or description
    String propellerType; ///< Propeller identifier or description
} TestMetadata_t;

typedef struct
{
    char format[3] = ".,";
} CsvFormat_t;

typedef struct
{
    TestMetadata_t metadata;
    CsvFormat_t csvFormat;
} TestConfiguration_t;

/**
 * @class ThrustTestController
 * @brief High-level controller for running automated thrust tests.
 *
 * Manages test sequences on a ThrustStand instance, including:
 *  - Step-based throttle control
 *  - Data acquisition
 *  - Progress tracking
 *  - Metadata management
 *  - CSV export
 */
class ThrustTestController
{
public:
    /**
     * @brief Construct a new ThrustTestController instance.
     *
     * @param stand Reference to the ThrustStand to control
     */
    ThrustTestController(ThrustStand &stand);

    void begin();

    /**
     * @brief Configure the timing of a test sequence.
     *
     * @param totalSteps Total number of throttle steps
     * @param stepAccelTimeMs Duration of acceleration for each step in milliseconds
     * @param stepTimeMs Duration of steady throttle per step in milliseconds
     * @param decelTimeMs Duration of deceleration after the test in milliseconds
     */
    void set_timing(
        uint32_t totalSteps,
        uint32_t stepAccelTimeMs,
        uint32_t stepTimeMs,
        uint32_t decelTimeMs);

    /**
     * @brief Start a new thrust test sequence.
     *
     * Initializes the step counter and starts data accumulation.
     *
     * @return true if the test was successfully started
     */
    bool startTest();

    /**
     * @brief Execute one iteration of the test sequence.
     *
     * Should be called repeatedly from the main loop.
     * Advances steps, updates throttle, and records sensor data.
     */
    void runTest();

    /**
     * @brief Stop the currently running test.
     *
     * Optionally stores the recorded test data.
     *
     * @param storeTestData If true, persist test data to storage
     */
    void stopTest(bool storeTestData = false);

    /**
     * @brief Check if a test is currently running.
     *
     * @return true if test is active
     */
    bool isTestRunning() const { return _status.running; }

    /**
     * @brief Set descriptive metadata for the test.
     *
     * @param motor Motor description or identifier
     * @param esc ESC description or identifier
     * @param prop Propeller description or identifier
     * @return true if metadata was set successfully
     */
    bool setMetadata(const String &motor, const String &esc, const String &prop);

    /**
     * @brief Array of recorded data points for the test (0-100%/20=5% steps on thrust testing)
     *
     * Indexed by step number (0–20).
     */
    test_data_accu_t test_data[21] = {};

    /**
     * @brief Get the current test status.
     *
     * @return Reference to current TestStatus_t structure
     */
    const TestStatus_t &getStatus() const { return _status; }

    /**
     * @brief Get the current test metadata.
     *
     * @return Reference to current TestMetadata_t structure
     */
    const TestMetadata_t &getMetadata() const { return _config.metadata; }

    void writeCsvMetadata(File &f);

    bool setCsvFormat(const char *format, bool saveConfigFlag = true);
    const CsvFormat_t &getCsvFormat() const { return _config.csvFormat; };

    /**
     * @brief Export recorded test data to LittleFS in CSV format.
     *
     * @param path File path to save CSV data
     * @param numberOfRecords Number of steps/records to write
     * @return true if CSV was successfully written
     */
    bool exportMeanCsv(const char *path, uint32_t numberOfRecords, const char *csvFormat = ".,");

    bool exportStatisticsCsv(const char *path,
                             uint32_t numberOfRecords,
                             const char *csvFormat = ".,");

private:
    static const char *TAG; ///< Logging tag
    ThrustStand &stand;     ///< Reference to the controlled ThrustStand

    uint32_t _max_total_steps = 20;      ///< Maximum number of throttle steps
    uint32_t _step_accel_time_ms = 1000; ///< Step acceleration time in ms
    uint32_t _step_time_ms = 2000;       ///< Step steady-state time in ms
    uint32_t _decel_time_ms = 1000;      ///< Deceleration time in ms

    unsigned long _step_start_ts = 0;     ///< Timestamp when current step started
    unsigned long _target_reached_ts = 0; ///< Timestamp when target throttle reached

    bool _accumulating = false; ///< Flag for ongoing data accumulation

    TestStatus_t _status = {false, 0, 0, 0.0f}; ///< Current test status
    TestConfiguration_t _config{};              ///< Current test metadata

    /**
     * @brief Update the progress percentage based on current step.
     */
    void updateProgress();

    static void printFloat(File &f, float value, int precision, char decimalSep);

    static void printStats(File &f,
                           const sensor_stats_t &s,
                           uint8_t decimals,
                           char decimalSep,
                           char fieldSep);

    /**
     * @brief Load configuration from non-volatile storage.
     */
    void loadConfig();

    /**
     * @brief Save configuration to non-volatile storage.
     */
    void saveConfig();
};

#endif // THRUST_TEST_CONTROLLER_H
