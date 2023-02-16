#pragma once

#include "concurrency/NotifiedWorkerThread.h"

#include "Sensor/GS1LFSensor.h"

#define GEOPHONE_MODULE_SAMPLES 4096

class GeophoneModule : public concurrency::NotifiedWorkerThread {
public:
    GeophoneModule();

    virtual void onNotify(uint32_t notification) override;
private:
    void collectData();
    void analyzeData(size_t numSamples);
private:
    bool dataCollectionStarted{};
    GS1LFSensor gs1lfSensor;
    size_t consecutiveBelowThreshold{};

    float rawData[GEOPHONE_MODULE_SAMPLES];
    float fftData[GEOPHONE_MODULE_SAMPLES];
    size_t dataIndex{};

    const size_t samplingFrequency{860};
    const double lowThreshold{0.5}; 
    const double highThreshold{1.0};
    const size_t maxConsecutiveBelowThreshold{10};
};

extern GeophoneModule* geophoneModule;
