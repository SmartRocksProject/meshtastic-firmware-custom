#include "GS1LFSensor.h"

#include <Wire.h>

#include "mesh/generated/meshtastic/telemetry.pb.h"
#include "main.h"

bool GS1LFSensor::setup(double lowThreshold, double highThreshold) {
    LOG_INFO("Init sensor: GS1LFSensor\n");
    if(!hasSensor()) {
        LOG_INFO("Could not find sensor: GS1LFSensor\n");
        return false;
    }

    if(!ADS.begin(I2C_SDA, I2C_SCL)) {
        LOG_INFO("Could not find sensor: GS1LFSensor\n");
        return false;
    }

    ADS.setGain(1);
    ADS.setDataRate(7);
    setContinuousMode(false);

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

void GS1LFSensor::setContinuousMode(bool continuousMode) {
    ADS.setMode(!continuousMode);
    if(continuousMode) {
        ADS.requestADC(0);
    }
}

bool GS1LFSensor::hasSensor() {
    // Check I2C wire for GS1LFSensor address
    Wire.beginTransmission(GS1LF_ADDR);
    if(Wire.endTransmission() == 0) {
        LOG_DEBUG("GS1LFSensor found at address 0x%x\n", GS1LF_ADDR);
        return true;
    }
    return false;
}

bool GS1LFSensor::readVoltage(float& out_voltage) {
    float voltage = ADS.toVoltage(ADS.getValue());
    if(voltage == ADS1X15_INVALID_VOLTAGE) {
        return false;
    }
    out_voltage = voltage;
    //LOG_INFO("GS1LFSensor::readVoltage: %f Volts\n", voltage);
    return true;
}
