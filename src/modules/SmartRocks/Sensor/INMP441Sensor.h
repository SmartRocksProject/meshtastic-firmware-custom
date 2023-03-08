#pragma once

class INMP441Sensor {
public:
    INMP441Sensor() = default;
    bool setup();
    bool readSample(uint8_t* sample);
    size_t readSamples(uint8_t* samples, size_t numSamples);
};