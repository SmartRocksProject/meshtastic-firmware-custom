#pragma once

#include <esp_pthread.h>

#include "concurrency/NotifiedWorkerThread.h"
#include "concurrency/Lock.h"

#include "Sensor/GS1LFSensor.h"
#include "Sensor/INMP441Sensor.h"

#define GEOPHONE_MODULE_SAMPLES 4096

class ActivityMonitorModule : public concurrency::NotifiedWorkerThread {
public:
    ActivityMonitorModule();
    ~ActivityMonitorModule();

    virtual void onNotify(uint32_t notification) override;

    TaskHandle_t runningTaskHandle{};
private:
    void collectData();
    void analyzeData();

    void collectGeophoneData();
    void collectMicrophoneData();

    static void geophoneCollectThread(void* p);
    static void microphoneCollectThread(void* p);
private:
    concurrency::Lock geophoneCollecting{};
    concurrency::Lock microphoneCollecting{};
    concurrency::Lock collectionFlagLock{};
    bool dataCollectionStarted{};

    struct GeophoneSensorData {
        GS1LFSensor gs1lfSensor;

        float* inputData;
        float* outputData;

        const double samplingFrequency{860.0};
        const double lowThreshold{0.3};
        const double highThreshold{1.0};
        const double amplitudeThreshold{1.0};
        const struct freqRange {
            double low;
            double high;
        } frequencyRangeThreshold{.low = 0.0, .high = 100.0};
    } geophoneSensorData;

    struct MicrophoneSensorData {
        INMP441Sensor inmp441Sensor;
        
        float* inputData;
        float* outputData;

        const double amplitudeThreshold{1.0};
        const struct freqRange {
            double low;
            double high;
        } frequencyRangeThreshold{.low = 0.0, .high = 100.0};
    } microphoneSensorData;
};

extern ActivityMonitorModule* activityMonitorModule;
