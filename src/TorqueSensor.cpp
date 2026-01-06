#include "TorqueSensor.h"
#include <esp_log.h>

static const char *initPhaseToString(TorqueInitPhase phase)
{
    switch (phase)
    {
    case TorqueInitPhase::IDLE:
        return "IDLE";
    case TorqueInitPhase::START_LC1:
        return "START_LC1";
    case TorqueInitPhase::WAIT_LC1:
        return "WAIT_LC1";
    case TorqueInitPhase::START_LC2:
        return "START_LC2";
    case TorqueInitPhase::WAIT_LC2:
        return "WAIT_LC2";
    case TorqueInitPhase::DONE:
        return "DONE";
    case TorqueInitPhase::ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

TorqueSensor::TorqueSensor(uint8_t doutPin1, uint8_t sckPin1,
                           uint8_t doutPin2, uint8_t sckPin2)
    : _lc1(doutPin1, sckPin1),
      _lc2(doutPin2, sckPin2)
{
}

bool TorqueSensor::begin(float cal1, float cal2, float distanceMM)
{
    if (isReady())
    {
        ESP_LOGI(TAG, "HX711 Torque Sensor Loadcells already initialized and ready.");
        return true;
    }

    ESP_LOGI(TAG, "Initialize HX711 Torque Loadcells ... initPhase=%s",
             initPhaseToString(_initPhase));
    if (getState() == SensorState::SENSOR_UNINITIALIZED)
    {
        setState(SensorState::SENSOR_INITIALIZING);
        _initPhase = TorqueInitPhase::IDLE;
    }

    unsigned long stabilizingTime = 3000;
    bool tare = true;

    if (_initPhase == TorqueInitPhase::IDLE)
    {
        _initPhase = TorqueInitPhase::START_LC1;
        _lc1.begin();
        _lc1.setGain(128);
        _lc2.begin();
        _lc2.setGain(128);
    }

    if (_initPhase == TorqueInitPhase::START_LC1)
    {
        _lc1.start(stabilizingTime, tare);
        _initPhase = TorqueInitPhase::WAIT_LC1;
    }

    if (_initPhase == TorqueInitPhase::WAIT_LC1)
    {
        if (_lc1.getTareTimeoutFlag())
        {
            ESP_LOGE(TAG, "Timeout, check MCU => HX711 Torque No.1 wiring and pin designations");
            return false;
        }
        else
        {
            _initPhase = TorqueInitPhase::START_LC2;
        }
    }

    if (_initPhase == TorqueInitPhase::START_LC2)
    {
        _lc2.start(stabilizingTime, tare);
        _initPhase = TorqueInitPhase::WAIT_LC2;
    }

    if (_initPhase == TorqueInitPhase::WAIT_LC2)
    {
        if (_lc2.getTareTimeoutFlag())
        {
            ESP_LOGE(TAG, "Timeout, check MCU => HX711 Torque No.2 wiring and pin designations");
            return false;
        }
        else
        {
            _initPhase = TorqueInitPhase::DONE;
        }
    }

    //        setState(SensorState::SENSOR_ERROR);
    //        incrementError();
    //        return false;

    if (_initPhase == TorqueInitPhase::DONE)
    {
        setCalibration(cal1, cal2, distanceMM);
        setDataValid(false);

        setState(SensorState::SENSOR_READY);
        ESP_LOGI(TAG, "HX711 Torque Loadcells initialized.");
        return true;
    }
    ESP_LOGE(TAG, "HX711 Torque Loadcells This line should never be called.");
    return false;
}

void TorqueSensor::setCalibration(float cal1, float cal2, float distanceMM)
{
    if (cal1 != 0.f)
    {
        _calibration.cal1 = cal1;
        _lc1.setCalFactor(cal1);
    }
    if (cal2 != 0.f)
    {
        _calibration.cal2 = cal2;
        _lc2.setCalFactor(cal2);
    }
    if (distanceMM != -1.f)
    {
        _calibration.distanceMM = distanceMM;
    }
}

bool TorqueSensor::update()
{
    if (!isReady())
        return false;

    uint8_t updated1 = _lc1.update();
    uint8_t updated2 = _lc2.update();

    if ((updated1 > 0) && (updated2 > 0))
    {
        _lc1_load = _lc1.getData();
        _lc2_load = _lc2.getData();

        setDataValid(true);
        ++_updateCount;
        return true;
    }

    // incrementError();
    // setDataValid(false);

    // Optional escalation
    // if (getErrorCount() > 5)
    //    setState(SensorState::SENSOR_ERROR);

    return false;
}

void TorqueSensor::tare()
{
    if (!isReady())
        return;
    ESP_LOGI("TorqueSensor", "Taring sensor ...");
    _lc1.tareNoDelay();
    _lc2.tareNoDelay();
    ESP_LOGI("TorqueSensor", "done");
}
