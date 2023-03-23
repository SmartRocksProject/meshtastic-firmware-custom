#pragma once

#include "configuration.h"

class INMP441Sensor {
public:
    INMP441Sensor() = default;
    bool setup(int sampleRate);
    bool readSample(uint8_t* sample);
    size_t readSamples(uint8_t* samples, size_t numSamples);
};