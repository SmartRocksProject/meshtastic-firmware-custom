#pragma once

#include "../mesh/generated/telemetry.pb.h"
#include "TelemetrySensor.h"

#include "ADS1X15.h"

class GS1LFSensor : virtual public TelemetrySensor {
private:
    ADS1115 ADS{0x48, &Wire1};

protected:
    virtual void setup() override;

public:
    GS1LFSensor();
    virtual int32_t runOnce() override;
    virtual bool getMetrics(Telemetry *measurement) override;
};    
