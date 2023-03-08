#include "INMP441Sensor.h"

#include "configuration.h"
#include <I2S.h>

bool INMP441Sensor::setup() {
    I2S.setAllPins(I2S_SCK, I2S_WS, I2S_SD, I2S_SD, I2S_SD);
    if(!I2S.begin(I2S_PHILIPS_MODE, 8000, 16)) {
        return false;
    }
    return true;
}

bool INMP441Sensor::readSample(uint8_t* sample) {
    return readSamples(sample, 1) == 1;
}

size_t INMP441Sensor::readSamples(uint8_t* samples, size_t numSamples) {
    return I2S.readBytes(samples, numSamples);
}