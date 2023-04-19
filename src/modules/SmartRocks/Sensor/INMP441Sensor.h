#pragma once

#include "configuration.h"

class INMP441Sensor {
public:
    INMP441Sensor() = default;
    bool setup(uint32_t sampleRate, int bufferLen);
    int readSample();
    
    // Size must be equal to bufferLen passed to setup
    void readSamples(int16_t* samples);
private:
    int _bufferLen{};
};