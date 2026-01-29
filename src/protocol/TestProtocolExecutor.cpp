/*
 * SPDX-FileCopyrightText: 2026 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "protocol/TestProtocolExecutor.h"
#include "core/ThrustStand.h"
#include <Arduino.h>
#include <esp_log.h>
#include <Preferences.h>

const char *TestProtocolExecutor::TAG = "TestProtocolExecutor";
static Preferences prefs;

static constexpr uint32_t TARGET_WAIT_TIMEOUT_MS = 2000;

TestProtocolExecutor::TestProtocolExecutor(ThrustStand &stand)
    : _stand(stand)
{
}

bool TestProtocolExecutor::begin()
{
    loadConfig();
    return true;
}

void TestProtocolExecutor::setRecorder(TestDataRecorder *recorder)
{
    _recorder = recorder;
}

void TestProtocolExecutor::setTestLimits(ProtocolLimits limits)
{
    // value of 0 indicating that it is not set
    if (limits.maxCurrentA > 0.)
        _stand.setCurrentLimitA(limits.maxCurrentA, SafetyTripSource::TEST_LIMIT);

    if (limits.maxTempC > 0.)
        _stand.setTemperatureLimitC(limits.maxTempC, SafetyTripSource::TEST_LIMIT);

    if (limits.maxVoltageV > 0.)
        _stand.setVoltageLimitMaxV(limits.maxVoltageV, SafetyTripSource::TEST_LIMIT);

    if (limits.minVoltageV > 0.)
        _stand.setVoltageLimitMinV(limits.minVoltageV, SafetyTripSource::TEST_LIMIT);

    if (limits.maxThrustGF > 0.)
        _stand.setThrustLimitGF(limits.maxThrustGF, SafetyTripSource::TEST_LIMIT);

    //    limits.maxDurationS check is handled in update() function
}

bool TestProtocolExecutor::isRunning() const
{
    return _protocolExecState.execState == ExecState::StepInit ||
           _protocolExecState.execState == ExecState::StepRunning;
}

ProtocolExecStatus TestProtocolExecutor::getStatus()
{
    updateProgress();
    return _protocolExecState;
};

ProtocolError TestProtocolExecutor::getLastError() const
{
    return _lastError;
}

bool TestProtocolExecutor::setRunContext(const String &motor,
                                         const String &esc,
                                         const String &prop,
                                         const String &protocolID,
                                         const String &protocolVersion,
                                         const char *csvFormat)

{
    if (isRunning())
        return false;

    if (motor.isEmpty() || prop.isEmpty())
        return false;

    _runContext.motorType = motor;
    _runContext.escType = esc;
    _runContext.propellerType = prop;
    _runContext.protocolID = protocolID;
    _runContext.protocolVersion = protocolVersion;

    if (csvFormat && strlen(csvFormat) == 2)
    {
        _runContext.csvFormat[0] = csvFormat[0];
        _runContext.csvFormat[1] = csvFormat[1];
        _runContext.csvFormat[2] = '\0';
    }
    saveConfig();
    return true;
}

void TestProtocolExecutor::resetInternalState()
{
    _steady.phase = SteadyPhase::WaitingForTarget;
    _steady.phaseStartMs = 0;

    _sweep.index = 0;
    _sweep.totalSteps = 0;
    _sweep.target = 0.0f;

    _currentThrottle = 0.0f;
}

StartError TestProtocolExecutor::start(const TestProtocol &protocol)
{
    resetInternalState();

    if (isRunning())
        return StartError::AlreadyRunning;
    if (_runContext.motorType.isEmpty())
        return StartError::MissingMotorType;

    if (_runContext.propellerType.isEmpty())
        return StartError::MissingPropType;

    ThrustStandState standState = _stand.getState();

    if (standState.motorMountState != MotorESC::EscState::ARMED)
        return StartError::MotorNotArmed;

    _proto = protocol;

    _runContext.protocolID = _proto.id;
    //    _runContext.protocolName = _proto.protocolName;
    _runContext.protocolVersion = _proto.version;

    if (!_stand.tareSensors())
        return StartError::TareFailed;

    _stand.setControlMode(ThrottleControlMode::AUTOMATIC);
    setTestLimits(_proto.limits);

    _protocolExecState.progressUnitsTotal = _proto.unitsTotal;
    _protocolExecState.progressUnitsDone = 0;

    _protocolExecState.currentStepIndex = 0;
    _protocolExecState.durationS = 0.;
    _protocolExecState.estDurationS = _proto.estDurationS;

    _protocolExecState.totalSteps = _proto.steps.size();
    _currentThrottle = 0.0f;
    _lastError = ProtocolError::Ok;

    ESP_LOGI(
        TAG,
        "Limits {maxCurrentA=%.2f maxTempC=%.2f maxVoltageV=%.2f minVoltageV=%.2f maxDurationS=%u "
        "maxThrustGF=%.2f}",
        _proto.limits.maxCurrentA,
        _proto.limits.maxTempC,
        _proto.limits.maxVoltageV,
        _proto.limits.minVoltageV,
        _proto.limits.maxDurationS,
        _proto.limits.maxThrustGF);

    _testStartMs = millis();
    _protocolExecState.execState = ExecState::StepInit;

    if (_recorder)
        _recorder->reset(_testStartMs);

    ESP_LOGI(TAG, "Test %s started with %u steps", _proto.name, (unsigned)_proto.steps.size());
    return StartError::Ok;
}

void TestProtocolExecutor::stop(ExecState finalState, bool storeTestData)
{
    _stand.setControlMode(ThrottleControlMode::MANUAL);
    if (!isRunning())
        return;

    _stand.setThrottle(0., ThrottleSource::TEST);

    _protocolExecState.execState = finalState;

    _protocolExecState.safetyState = _stand.getSafetyState();
    if (_protocolExecState.safetyState.tripped)
    {
        switch (_protocolExecState.safetyState.source)
        {
        case SafetyTripSource::CORE_LIMIT:
            _lastError = ProtocolError::SafetyTriggerRaised;
            _protocolExecState.execState = ExecState::Aborted;
            break;

        case SafetyTripSource::TEST_LIMIT:
        default:
            _protocolExecState.execState = ExecState::Completed;
            break;
        }
    }

    if (_recorder)
    {
        if (_recorder->isRecording())
        {
            ESP_LOGI(TAG, "Recording is still recording->force to stop");
            _recorder->stop();
        }
    }

    _protocolExecState.execState = finalState;
    _protocolExecState.running = false;

    if (storeTestData)
    {
        if (_recorder)
        {
            ESP_LOGI(TAG, "Test completed and test data stored");
            _recorder->exportMeanCsv("/last_test_mean.csv", _runContext, _protocolExecState);
            _recorder->exportStatisticsCsv("/last_test_stats.csv", _runContext, _protocolExecState);
        }
        else
        {
            ESP_LOGW(TAG, "No DataRecorder assigned->no test data stored");
        }
    }
    else
    {
        ESP_LOGI(TAG, "Test stopped and no test data stored");
    }
}

// Run the test (call in main loop or task)
void TestProtocolExecutor::update()
{
    if (!isRunning())
        return;

    _protocolExecState.durationS =
        (millis() - _testStartMs) * 0.001f;

    // Safety always wins
    if (_stand.isSafetyTriggered())
    {
        ESP_LOGI(TAG, "isSafetyTriggered is true");
        stop(ExecState::Aborted, true);
        return;
    }

    if (_recorder)
    {
        if (_recorder->isFull())
        {
            ESP_LOGI(TAG, "_recorder->isFull()");

            abort(ProtocolError::DataRecorderFull);
            return;
        }
    }

    if (_protocolExecState.durationS > _proto.limits.maxDurationS)
    {
        ESP_LOGI(TAG, "test ran longer than allowed");

        abort(ProtocolError::TotalDurationExceeded);
        return;
    }

    switch (_protocolExecState.execState)
    {
    case ExecState::StepInit:
        initStep();
        break;

    case ExecState::StepRunning:
        runStep();
        break;

    default:
        break;
    }
}

void TestProtocolExecutor::initStep()
{
    if (_protocolExecState.currentStepIndex >= _proto.steps.size())
    {
        _stand.setThrottle(0.0f, ThrottleSource::TEST);
        ESP_LOGI(TAG, "_protocolExecState.currentStepIndex %i >= %i _proto.steps.size() ", _protocolExecState.currentStepIndex, _proto.steps.size());
        stop(ExecState::Completed, true);
        return;
    }

    _currentStep = &_proto.steps[_protocolExecState.currentStepIndex];
    _currentMotionCfg = _currentStep->motion;
    _currentMeasureCfg = _currentStep->measure;

    ESP_LOGI(
        TAG,
        "Step[%u] type=%u | throttleFrom=%.3f throttleTo=%.3f rampRate=%.3f stepPct=%.3f dwellMs=%u | "
        "motion{smooth=%d accel=%u decel=%u settle=%u} | "
        "measure{enabled=%d completion=%u intervalS=%u windowMs=%u}",
        _protocolExecState.currentStepIndex,
        static_cast<uint8_t>(_currentStep->type),

        _currentStep->throttleFrom,
        _currentStep->throttleTo,
        _currentStep->rampRatePctPerS,
        _currentStep->stepPct,
        _currentStep->dwellMs,

        _currentStep->motion.smooth,
        _currentStep->motion.accelTimeMs,
        _currentStep->motion.decelTimeMs,
        _currentStep->motion.settleTimeMs,

        _currentStep->measure.enabled,
        static_cast<uint8_t>(_currentStep->measure.completion),
        _currentStep->measure.intervalS,
        _currentStep->measure.windowMs);

    _stepStartMs = millis();

    _sweep = StepSweepRuntime{};
    _steady = SteadyStateRunner{};
    resetSteady();

    switch (_currentStep->type)
    {
    case TestStepType::Set:
    case TestStepType::Hold:
    {
        _currentThrottle = _currentStep->throttleTo;

        _stand.setThrottle(
            _currentThrottle,
            _currentMotionCfg.smooth,
            _currentMotionCfg.accelTimeMs,
            ThrottleSource::TEST);

        break;
    }

    case TestStepType::Ramp:
    // how to deal with rampRatePctPerS
    case TestStepType::StepSweep:
    {
        _sweep.totalSteps = static_cast<uint16_t>(
                                round(fabs(_currentStep->throttleTo - _currentStep->throttleFrom) / _currentStep->stepPct)) +
                            1;

        _sweep.index = 0;
        _sweep.target = _currentStep->throttleFrom;

        ESP_LOGI(TAG, " _sweep.totalSteps %i, _sweep.target: %f", _sweep.totalSteps, _sweep.target);

        _stand.setThrottle(
            _sweep.target,
            _currentMotionCfg.smooth,
            _currentMotionCfg.accelTimeMs,
            ThrottleSource::TEST);

        break;
    }

    default:
        ESP_LOGI(TAG, "Abort: UnknownStepType SetpIndex: %i stepType: %i", _protocolExecState.currentStepIndex, static_cast<uint8_t>(_currentStep->type));
        abort(ProtocolError::UnknownStepType);
        return;
    }

    _protocolExecState.execState = ExecState::StepRunning;
    // logProtoStep1("leaving initStep");
}

void TestProtocolExecutor::runStep()
{
    uint32_t now = millis();
    uint32_t elapsedMs = (now - _stepStartMs);

    // ESP_LOGI(TAG,
    //          "_currentStep->type raw=%u",
    //          static_cast<uint8_t>(_currentStep->type));

    switch (_currentStep->type)
    {
    case TestStepType::Set:
    case TestStepType::Hold:
    {
        // logProtoStep1("runStep->runSteadyState");
        if (!runSteadyState(
                _currentStep->dwellMs,
                _currentMotionCfg,
                _currentMeasureCfg))
            return;
        // logProtoStep1("TestStepType::Set before _protocolExecState.currentStepIndex++ ");
        _protocolExecState.currentStepIndex++;
        _protocolExecState.progressUnitsDone++;
        _protocolExecState.execState = ExecState::StepInit;
        return;
    }

    case TestStepType::Ramp:
    {
        // Calculate ramp target
        float delta = _currentStep->rampRatePctPerS * elapsedMs * 0.001f;
        float target = (_currentStep->throttleTo > _currentStep->throttleFrom)
                           ? _currentStep->throttleFrom + delta
                           : _currentStep->throttleFrom - delta;

        bool done = (_currentStep->throttleTo > _currentStep->throttleFrom && target >= _currentStep->throttleTo) ||
                    (_currentStep->throttleTo < _currentStep->throttleFrom && target <= _currentStep->throttleTo);

        // --- Measurement handling ---
        if (!runSteadyState(
                0, // ramp has no fixed dwell, just measure continuously
                _currentMotionCfg,
                _currentMeasureCfg))
        {
            // Still measuring, continue ramp
            _currentThrottle = constrain(target, 0.0f, 100.0f);
            _stand.setThrottle(
                _currentThrottle,
                _currentMotionCfg.smooth,
                _currentMotionCfg.accelTimeMs,
                ThrottleSource::TEST);
            return;
        }

        // Ramp finished: set final throttle
        _currentThrottle = _currentStep->throttleTo;
        _stand.setThrottle(
            _currentThrottle,
            _currentMotionCfg.smooth,
            _currentMotionCfg.decelTimeMs,
            ThrottleSource::TEST);

        _protocolExecState.currentStepIndex++;
        _protocolExecState.progressUnitsDone++;
        _protocolExecState.execState = ExecState::StepInit;
        return;
    }

    case TestStepType::StepSweep:
    {
        if (!runSteadyState(
                _currentStep->dwellMs,
                _currentMotionCfg,
                _currentMeasureCfg))
            return;

        // One sweep point completed
        ESP_LOGI(TAG, "One sweep point completed");
        _sweep.index++;
        _protocolExecState.progressUnitsDone++;

        if (_sweep.index >= _sweep.totalSteps)
        {
            _currentThrottle = _currentStep->throttleTo;
            _protocolExecState.currentStepIndex++;
            _protocolExecState.execState = ExecState::StepInit;
            return;
        }

        // Advance throttle
        float stepSize = _currentStep->stepPct;
        float dir = (_currentStep->throttleTo > _currentStep->throttleFrom) ? 1.0f : -1.0f;

        _sweep.target =
            _currentStep->throttleFrom + dir * stepSize * _sweep.index;

        _sweep.target = constrain(_sweep.target, 0.0f, 100.0f);

        resetSteady(); // CRITICAL: reset steady BEFORE commanding new throttle

        _stand.setThrottle(
            _sweep.target,
            _currentMotionCfg.smooth,
            _currentMotionCfg.accelTimeMs,
            ThrottleSource::TEST);

        return;
    }

    default:
        ESP_LOGI(TAG, "Abort: UnknownStepType");
        abort(ProtocolError::UnknownStepType);
        break;
    }
}

void TestProtocolExecutor::resetSteady()
{
    _steady.phase = SteadyPhase::WaitingForTarget;
    _steady.phaseStartMs = 0; // millis();
}

bool TestProtocolExecutor::runSteadyState(
    uint32_t dwellMs,
    const MotionConfig &motion,
    const MeasurementConfig &measure)
{
    uint32_t now = millis();

    switch (_steady.phase)
    {
    case SteadyPhase::WaitingForTarget:
        if (_steady.phaseStartMs == 0)
        {
            _steady.phaseStartMs = now;
            return false; // guarantee at least one loop delay
        }
        // logProtoStep1("SteadyPhase::WaitingForTarget before MotorAtTargetSpeed ");
        if (_stand.isMotorAtTargetSpeed())
        {

            _steady.phase = SteadyPhase::Settling;
            _steady.phaseStartMs = now;
            return false;
        }

        // ⏱ timeout fallback
        if (now - _steady.phaseStartMs > TARGET_WAIT_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "Target speed timeout, forcing settle");
            _steady.phase = SteadyPhase::Settling;
            _steady.phaseStartMs = now;
            return false;
        }

        return false;

    case SteadyPhase::Settling:
        if (now - _steady.phaseStartMs < motion.settleTimeMs)
            return false;
        // logProtoStep1("SteadyPhase::Settling before recorder.start ");
        if (_recorder)
            _recorder->trigger(measure);

        _steady.phase = SteadyPhase::Measuring;
        _steady.phaseStartMs = now;
        return false;

    case SteadyPhase::Measuring:
    {

        // Always check if dwellS has elapsed
        bool dwellElapsed = (now - _steady.phaseStartMs >= dwellMs);

        bool measurementDone = false;

        if (_recorder)
        {
            switch (measure.completion)
            {
            case MeasurementCompletion::OnStop:
                if (dwellElapsed)
                {
                    _recorder->stop();
                    measurementDone = true;
                }
                break;

            case MeasurementCompletion::FixedDuration:
                // Recorder stops automatically after windowMs
                // We consider measurement done when recorder is no longer recording
                if (!_recorder->isRecording())
                {
                    measurementDone = true;
                }
                break;

            case MeasurementCompletion::Periodic:
                // Periodic continues independently until protocol stops
                if (dwellElapsed)
                    measurementDone = true; // Phase can move on
                break;
            }
        }
        else
        {
            // No recorder, just rely on dwell
            measurementDone = dwellElapsed;
        }

        if (measurementDone)
        {
            // logProtoStep1("SteadyPhase::Measuring - measurementDone");
            _steady.phase = SteadyPhase::Done;
            return true;
        }

        return false;
    }

    case SteadyPhase::Done:
        return true;

    default:
        return false;
    }
}

void TestProtocolExecutor::abort(ProtocolError reason)
{
    _lastError = reason;
    stop(ExecState::Aborted, true);
}

void TestProtocolExecutor::updateProgress()
{
    _protocolExecState.running = isRunning();
    if (_protocolExecState.progressUnitsTotal == 0)
    {
        _protocolExecState.progressPct = 0.0f;
        return;
    }

    if (_proto.estDurationS > 0.f)
    {
        _protocolExecState.progressPct = 100.f * _protocolExecState.durationS / _proto.estDurationS;
    }

    // if (_protocolExecState.execState > ExecState::StepRunning)
    //     _protocolExecState.progressPct = 100.f;

    _protocolExecState.progressPct = min(_protocolExecState.progressPct, 100.0f);
}

void TestProtocolExecutor::fillStatusJson(JsonDocument &doc)
{
    test_data_accu_t c = _stand.getAccuDataSet();
    ProtocolExecStatus ctx = getStatus();

    doc["steps_total"] = ctx.totalSteps;
    doc["unitsTotal"] = ctx.progressUnitsTotal;
    doc["running"] = ctx.running;
    if (ctx.execState != ExecState::Idle && ctx.totalSteps > 0)
    {
        doc["step"] = (ctx.currentStepIndex >= ctx.totalSteps) ? ctx.totalSteps : (ctx.currentStepIndex + 1);
        doc["unitsDone"] = (ctx.progressUnitsDone >= ctx.progressUnitsTotal) ? ctx.progressUnitsTotal : (ctx.progressUnitsDone + 1);
    }
    else
    {
        doc["step"] = 0;
        doc["unitsDone"] = 0;
    }

    doc["progress"] = ctx.progressPct;

    doc["thrust_samples"] = c.thrust.n;
    doc["durationS"] = ctx.durationS;
    doc["estDurationS"] = ctx.estDurationS;
    doc["execState"] = static_cast<uint8_t>(ctx.execState);
    doc["safetyState_tripped"] = ctx.safetyState.tripped;
}

void TestProtocolExecutor::fillConfigJson(JsonDocument &doc) const
{
    doc["motorType"] = _runContext.motorType;
    doc["escType"] = _runContext.escType;
    doc["propellerType"] = _runContext.propellerType;
    doc["csvFormat"] = _runContext.csvFormat;
    doc["protocolID"] = _runContext.protocolID;
    doc["protocolVersion"] = _runContext.protocolVersion;
}

void TestProtocolExecutor::loadConfig()
{
    float interimValue;
    prefs.begin("test", false); // false=write mode in case the values as not yet set the will be written via putFloat, ....

    if (!prefs.isKey("motorType"))
        prefs.putString("motorType", "Motor Type");
    if (!prefs.isKey("escType"))
        prefs.putString("escType", "ESC Type");
    if (!prefs.isKey("propellerType"))
        prefs.putString("propellerType", "Propeller Type");

    _runContext.motorType = prefs.getString("motorType");
    _runContext.escType = prefs.getString("escType");
    _runContext.propellerType = prefs.getString("propellerType");

    _runContext.protocolID = prefs.getString("protocolID", "");
    prefs.putString("protocolID", _runContext.protocolID);

    _runContext.protocolVersion = prefs.getString("protocolVersion", "");
    prefs.putString("protocolVersion", _runContext.protocolVersion);

    if (!prefs.isKey("csvFormat"))
        prefs.putString("csvFormat", ",;"); // German Excel Style

    String csvFormat = prefs.getString("csvFormat");
    if (csvFormat && strlen(csvFormat.c_str()) == 2)
    {
        _runContext.csvFormat[0] = csvFormat[0];
        _runContext.csvFormat[1] = csvFormat[1];
        _runContext.csvFormat[2] = '\0';
    }

    prefs.end();
}

void TestProtocolExecutor::saveConfig()
{
    prefs.begin("test", false);
    prefs.putString("motorType", _runContext.motorType);
    prefs.putString("escType", _runContext.escType);
    prefs.putString("propellerType", _runContext.propellerType);
    prefs.putString("protocolID", _runContext.protocolID);
    prefs.putString("protocolVersion", _runContext.protocolVersion);

    prefs.putString("csvFormat", _runContext.csvFormat);

    prefs.end();
}

void TestProtocolExecutor::logProtoStep1(const char *position)
{
    if (_proto.steps.size() <= 1)
    {
        ESP_LOGW(TAG, "%s proto[1] not available", position);
        return;
    }

    ESP_LOGI(
        TAG,
        "%s proto[1]: type=%u dwellMs=%u accel=%u",
        position,
        static_cast<uint8_t>(_proto.steps[1].type),
        _proto.steps[1].dwellMs,
        _proto.steps[1].motion.accelTimeMs);
}
