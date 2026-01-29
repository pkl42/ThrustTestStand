#include "protocol/TestProtocolValidator.h"
// #include "recorder/TestDataRecorder.h"
#include <cmath>

static constexpr float MIN_THROTTLE = 0.0f;
static constexpr float MAX_THROTTLE = 100.0f;

static constexpr float MAX_RAMP_RATE = 50.0f; // % per second
static constexpr uint32_t MAX_DWELL_MS = 600000;  // 10 minutes 60*60*1000 ms

ProtocolValidationResult validateProtocol(const TestProtocol &proto)
{
    ProtocolValidationResult res{};
    res.error = ProtocolError::Ok;
    res.totalDurationS = 0.0f;

    if (proto.steps.empty())
    {
        res.error = ProtocolError::EmptyProtocol;
        return res;
    }

    // proto.steps.size() does not need to correlate with MAX_RECORD_STEPS of dataRecorder, e.g. timeseries
    // need to be refined
    // if (proto.steps.size() > MAX_RECORD_STEPS)
    // {
    //     res.error = ProtocolError::TooManySteps;
    //     return res;
    // }

    for (const auto &step : proto.steps)
    {
        switch (step.type)
        {
        case TestStepType::Set:
        case TestStepType::Hold:
        {
            if (step.throttleTo < MIN_THROTTLE ||
                step.throttleTo > MAX_THROTTLE)
            {
                res.error = ProtocolError::ThrottleOutOfRange;
                return res;
            }


            if (step.dwellMs < 0 || step.dwellMs > MAX_DWELL_MS)
            {
                res.error = ProtocolError::DwellOutOfRange;
                return res;
            }

            res.totalDurationS += step.dwellMs/1000.;
            break;
        }

        case TestStepType::Ramp:
        {
            if (step.throttleFrom < MIN_THROTTLE ||
                step.throttleFrom > MAX_THROTTLE ||
                step.throttleTo < MIN_THROTTLE ||
                step.throttleTo > MAX_THROTTLE)
            {
                res.error = ProtocolError::ThrottleOutOfRange;
                return res;
            }

            if (step.rampRatePctPerS <= 0.0f)
            {
                res.error = ProtocolError::RampRateZero;
                return res;
            }

            if (step.rampRatePctPerS > MAX_RAMP_RATE)
            {
                res.error = ProtocolError::RateOutOfRange;
                return res;
            }

            float delta = fabs(step.throttleTo - step.throttleFrom);
            res.totalDurationS += delta / step.rampRatePctPerS;
            break;
        }

        default:
            res.error = ProtocolError::UnknownStepType;
            return res;
        }
    }

    if (proto.limits.maxDurationS > 0 &&
        res.totalDurationS > proto.limits.maxDurationS)
    {
        res.error = ProtocolError::TotalDurationExceeded;
        return res;
    }

    // Safety compatibility checks (soft for now)
    if (proto.limits.maxCurrentA > 0 &&
        proto.limits.maxCurrentA > 200.0f)
    {
        res.error = ProtocolError::UnsafeCurrentLimit;
        return res;
    }

    if (proto.limits.maxTempC > 0 &&
        proto.limits.maxTempC > 150.0f)
    {
        res.error = ProtocolError::UnsafeTemperatureLimit;
        return res;
    }

    return res;
}
