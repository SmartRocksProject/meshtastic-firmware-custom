#include "GS1LFSensor.h"

#include "mesh/generated/telemetry.pb.h"
#include "main.h"

bool GS1LFSensor::setup(double lowThreshold, double highThreshold) {
    DEBUG_MSG("Init sensor: GS1LFSensor\n");
    if(!hasSensor()) {
        DEBUG_MSG("Could not find sensor: GS1LFSensor\n");
        return false;
    }

    if(!ADS.begin(I2C_SDA, I2C_SCL)) {
        DEBUG_MSG("Could not find sensor: GS1LFSensor\n");
        return false;
    }

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

    // ALERT on/off thresholds
    float f = ADS.toVoltage(1);
    ADS.setComparatorThresholdLow(lowThreshold / f);
    ADS.setComparatorThresholdHigh(highThreshold / f);

    return true;
}

bool GS1LFSensor::hasSensor() {
    return TelemetrySensorType_GS1LF < sizeof(nodeTelemetrySensorsMap) && nodeTelemetrySensorsMap[TelemetrySensorType_GS1LF] > 0;
}

bool GS1LFSensor::readVoltage(float& out_voltage) {
    DEBUG_MSG("GS1LFSensor::readVoltage\n");
    float voltage = ADS.toVoltage(ADS.getValue());
    if(voltage == ADS1X15_INVALID_VOLTAGE) {
        return false;
    }
    out_voltage = voltage;
    return true;
}
