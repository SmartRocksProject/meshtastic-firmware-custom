#pragma once

#include "ADS1X15.h"
#include "configuration.h"

class GS1LFSensor {
public:
    GS1LFSensor() = default;
    bool setup(double lowThreshold, double highThreshold);
    bool readVoltage(float& measurement);

    void setContinuousMode(bool continuousMode);
private:
    bool hasSensor();
private:
    ADS1115 ADS{GS1LF_ADDR, &Wire1};
};
