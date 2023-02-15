#include "GS1LFSensor.h"

GS1LFSensor::GS1LFSensor()
    : TelemetrySensor(TelemetrySensorType_GS1LF, "Geophone") {}

void GS1LFSensor::setup() {}

int32_t GS1LFSensor::runOnce() {
    DEBUG_MSG("Init sensor: %s\n", sensorName);
    if (!hasSensor()) {
        return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
    }
    status = ADS.begin(I2C_SDA, I2C_SCL);

    ADS.setGain(1);
    ADS.setDataRate(7);
    ADS.setMode(0);
    ADS.requestADC(0);

    // Traditional mode (assert above high threshold, de-assert below low)
    ADS.setComparatorMode(0);

    // Active low ALERT
    ADS.setComparatorPolarity(0);

    ADS.setComparatorLatch(1);

    // Trigger ALERT after single conversion breaks threshold.
    ADS.setComparatorQueConvert(0);

    float f = ADS.toVoltage(1);
    ADS.setComparatorThresholdLow(1.234 / f);
    ADS.setComparatorThresholdHigh(3.142 / f);
    
    return initI2CSensor();
}

bool GS1LFSensor::getMetrics(Telemetry *measurement) {
    DEBUG_MSG("GS1LFSensor::getMetrics\n");
    measurement->variant.environment_metrics.voltage = ADS.toVoltage(ADS.getValue());
    if(measurement->variant.environment_metrics.voltage == ADS1X15_INVALID_VOLTAGE) {
        return false;
    }

    return true;
}
