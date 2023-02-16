// Must be defined first to prevent build errors (conflict with str being defined as a macro by Meshtastic libs)
#include "yashSanghvi_FFT/FFT.h"

#include "GeophoneModule.h"

GeophoneModule* geophoneModule;

static void sensorInterrupt() {
    geophoneModule->notify(1, false);
}

GeophoneModule::GeophoneModule()
    : concurrency::NotifiedWorkerThread("GeophoneModule")
{
    if(gs1lfSensor.setup(lowThreshold, highThreshold)) {
        attachInterrupt(0, sensorInterrupt, FALLING);
    }
}

void GeophoneModule::onNotify(uint32_t notification) {
    // Ignore alert if data collection ongoing.
    if(!dataCollectionStarted) {
        dataCollectionStarted = true;
        collectData();
    }
}

void GeophoneModule::collectData() {
    while(dataIndex < GEOPHONE_MODULE_SAMPLES || consecutiveBelowThreshold < maxConsecutiveBelowThreshold) {
        float voltage = 0.0f;
        if(!gs1lfSensor.readVoltage(voltage)) {
            DEBUG_MSG("Error when reading GS1LFSensor voltage!\n");
            dataCollectionStarted = false;
            return;
        }

        if(voltage > highThreshold) {
            rawData[dataIndex++] = voltage;
            consecutiveBelowThreshold = 0;
        } else {
            consecutiveBelowThreshold++;
        }

        delayMicroseconds(1e6 / samplingFrequency);
    }
    // Total samples collected.
    size_t totalSamples = dataIndex;

    consecutiveBelowThreshold = 0;
    dataIndex = 0;

    // Analyze and send data if event occured.
    analyzeData(totalSamples);

    // Reset collection flag.
    dataCollectionStarted = false;
}

void GeophoneModule::analyzeData(size_t numSamples) {
    //double totalTime = numSamples / samplingFrequency;

    //float maxMagnitude{};
    //float fundamentalFreq{};

    fft_config_t *real_fft_plan = fft_init(numSamples, FFT_REAL, FFT_FORWARD, rawData, fftData);
    fft_execute(real_fft_plan);

    // TODO: Analyze frequencies and send data
}
