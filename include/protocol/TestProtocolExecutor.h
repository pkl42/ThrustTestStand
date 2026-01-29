/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEST_PROTOCOL_EXECUTOR_H
#define TEST_PROTOCOL_EXECUTOR_H

#include <stdint.h>
#include "protocol/TestProtocol.h"
#include "protocol/ProtocolErrors.h"
#include "recorder/TestDataRecorder.h"
#include "core/ThrustStand.h"
#include <Arduino.h>
#include <esp_log.h>

class TestDataRecorder;

/**
 * @brief High-level execution state of a test protocol.
 *
 * Represents the coarse-grained lifecycle state of the
 * protocol executor.
 */
enum class ExecState : uint8_t
{
    Idle = 0,        ///< Not running
    StepInit = 1,    ///< Initializing a protocol step
    StepRunning = 2, ///< Executing a protocol step
    Completed = 3,   ///< Protocol finished successfully
    Aborted = 4,     ///< Protocol aborted by user
    Failed = 5       ///< Protocol failed due to error
};

/**
 * @brief Convert ExecState to human-readable string.
 *
 * @param state Execution state.
 * @return String representation.
 */
inline const char *execStateToString(ExecState state)
{
    switch (state)
    {
    case ExecState::Idle:
        return "Idle";
    case ExecState::StepInit:
        return "StepInit";
    case ExecState::StepRunning:
        return "StepRunning";
    case ExecState::Completed:
        return "Completed";
    case ExecState::Aborted:
        return "Aborted";
    case ExecState::Failed:
        return "Failed";
    default:
        return "Unknown";
    }
}

/**
 * @brief Aggregated protocol execution status.
 *
 * This structure is designed for UI, telemetry, and
 * progress reporting.
 */
struct ProtocolExecStatus
{
    bool running = false;          ///< True if protocol is active
    uint16_t currentStepIndex = 0; ///< Current step index
    uint16_t totalSteps = 0;       ///< Total number of steps

    float durationS = 0;             ///< Elapsed execution time (s)
    float estDurationS = 0;          ///< Estimated total duration (s)
    uint32_t progressUnitsDone = 0;  ///< Completed progress units
    uint32_t progressUnitsTotal = 0; ///< Total progress units
    float progressPct = 0;           ///< Progress in percent

    ExecState execState = ExecState::Idle; ///< Current execution state
    SafetyState safetyState;               ///< Current safety system state
};

/**
 * @brief Phases of steady-state step execution.
 */
enum class SteadyPhase
{
    Idle,
    WaitingForTarget,
    Settling,
    Measuring,
    Done
};

/**
 * @brief Runtime state for steady-state execution.
 */
struct SteadyStateRunner
{
    SteadyPhase phase = SteadyPhase::Idle; ///< Current steady-state phase
    uint32_t phaseStartMs = 0;             ///< Phase start timestamp
};

/**
 * @brief Phases of a sweep step execution.
 */
enum class SweepPhase
{
    WaitingForTarget, ///< Waiting for motor to reach target
    Settling,         ///< Target reached, waiting settle time
    Measuring         ///< Dwell and measurement phase
};

/**
 * @brief Runtime data for sweep step execution.
 */
struct StepSweepRuntime
{
    uint16_t index = 0;      ///< Current sweep index
    uint16_t totalSteps = 0; ///< Total sweep steps
    float target = 0.0f;     ///< Current target value
};

/**
 * @brief Possible errors when starting a protocol run.
 */
enum class StartError
{
    Ok,
    AlreadyRunning,
    MissingMotorType,
    MissingPropType,
    TareFailed,
    InvalidProtocol,
    SafetyLocked,
    MotorNotArmed
};

/**
 * @class TestProtocolExecutor
 * @brief Executes a TestProtocol on the thrust stand.
 *
 * The TestProtocolExecutor is responsible for:
 * - Validating preconditions before execution
 * - Sequencing protocol steps
 * - Driving the ThrustStand
 * - Coordinating measurement and recording
 * - Tracking execution progress and errors
 *
 * Execution is cooperative and non-blocking; update()
 * must be called periodically from the main loop().
 *
 * @section executor_thread_safety Thread / Execution Context Safety
 * - All public methods must be called from non-ISR context
 * - update() must be called from the main loop()
 */
class TestProtocolExecutor
{
public:
    /**
     * @brief Construct a protocol executor.
     *
     * @param stand Reference to the thrust stand hardware interface.
     */
    explicit TestProtocolExecutor(ThrustStand &stand);

    /**
     * @brief Initialize the executor.
     *
     * @return true on successful initialization.
     */
    bool begin();

    /**
     * @brief Start executing a test protocol.
     *
     * Performs validation, initializes runtime state,
     * and transitions to active execution.
     *
     * @param protocol Protocol definition to execute.
     * @return StartError describing success or failure reason.
     */
    StartError start(const TestProtocol &protocol);

    /**
     * @brief Stop protocol execution.
     *
     * @param finalState Final execution state.
     * @param storeTestData Whether to persist recorded data.
     */
    void stop(ExecState finalState, bool storeTestData = false);

    /**
     * @brief Periodic update function.
     *
     * Advances the protocol state machine and executes
     * the current step.
     */
    void update();

    /**
     * @brief Check whether a protocol is currently running.
     *
     * @return true if execution is active.
     */
    bool isRunning() const;

    /**
     * @brief Get current execution status.
     *
     * @return Snapshot of execution status.
     */
    ProtocolExecStatus getStatus();

    /**
     * @brief Get the last protocol error.
     *
     * @return Last ProtocolError.
     */
    ProtocolError getLastError() const;

    /**
     * @brief Attach a data recorder.
     *
     * @param recorder Recorder instance or nullptr to detach.
     */
    void setRecorder(TestDataRecorder *recorder);

    /**
     * @brief Set metadata describing the test run.
     *
     * @param motor Motor identifier.
     * @param esc ESC identifier.
     * @param prop Propeller identifier.
     * @param protocolID Protocol identifier.
     * @param protocolVersion Protocol version string.
     * @param csvFormat Optional CSV formatting override.
     *
     * @return true if context was accepted.
     */
    bool setRunContext(const String &motor,
                       const String &esc,
                       const String &prop,
                       const String &protocolID,
                       const String &protocolVersion,
                       const char *csvFormat = nullptr);

    /**
     * @brief Get the current test run context.
     *
     * @return Test run metadata.
     */
    TestRunContext getRunContext() const { return _runContext; }

    /**
     * @brief Fill execution status as JSON.
     *
     * @param doc Output JSON document.
     */
    void fillStatusJson(JsonDocument &doc);

    /**
     * @brief Fill available protocol list as JSON.
     *
     * @param doc Output JSON document.
     * @return true on success.
     */
    bool fillProtocolListJson(JsonDocument &doc) const;

    /**
     * @brief Fill configuration information as JSON.
     *
     * @param doc Output JSON document.
     */
    void fillConfigJson(JsonDocument &doc) const;

private:
    static const char *TAG; ///< Logging tag

    ThrustStand &_stand;                   ///< Thrust stand interface
    TestDataRecorder *_recorder = nullptr; ///< Optional data recorder
    TestProtocol _proto;                   ///< Active protocol

    ProtocolExecStatus _protocolExecState{}; ///< Execution status

    uint32_t _stepStartMs = 0;     ///< Step start timestamp
    uint32_t _testStartMs = 0;     ///< Test start timestamp
    float _currentThrottle = 0.0f; ///< Current throttle value

    const TestStep *_currentStep = nullptr; ///< Active protocol step
    MotionConfig _currentMotionCfg;         ///< Active motion configuration
    MeasurementConfig _currentMeasureCfg;   ///< Active measurement configuration
    StepSweepRuntime _sweep;                ///< Sweep runtime state
    SteadyStateRunner _steady;              ///< Steady-state runtime state

    ProtocolError _lastError = ProtocolError::Ok; ///< Last error
    TestRunContext _runContext;                   ///< Test run metadata

    void resetInternalState();
    void resetSteady();

    bool runSteadyState(uint32_t dwellMs,
                        const MotionConfig &motion,
                        const MeasurementConfig &measure);

    void setTestLimits(ProtocolLimits limits);
    void initStep();
    void runStep();
    void abort(ProtocolError reason);

    void loadConfig();
    void saveConfig();

    /**
     * @brief Update progress counters and percentage.
     */
    void updateProgress();

    /**
     * @brief Log protocol step transitions.
     *
     * @param position Textual step position indicator.
     */
    void logProtoStep1(const char *position);
};

#endif // TEST_PROTOCOL_EXECUTOR_H
