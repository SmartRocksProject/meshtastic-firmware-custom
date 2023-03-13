// Must be defined first to prevent build errors (conflict with str being defined as a macro by Meshtastic libs)
#include "yashSanghvi_FFT/FFT.h"

#include "ActivityMonitorModule.h"

//#include <arduinoFFT.h>

ActivityMonitorModule* activityMonitorModule;

static void sensorInterrupt() {
    activityMonitorModule->notify(1, false);
}

ActivityMonitorModule::ActivityMonitorModule()
    : concurrency::NotifiedWorkerThread("ActivityMonitorModule")
{
    if(geophoneSensorData.gs1lfSensor.setup(geophoneSensorData.lowThreshold, geophoneSensorData.highThreshold)) {
        geophoneSensorData.inputData = (float*) ps_malloc(sizeof(float) * GEOPHONE_MODULE_SAMPLES);
        geophoneSensorData.outputData = (float*) ps_malloc(sizeof(float) * GEOPHONE_MODULE_SAMPLES);
        
        attachInterrupt(0, sensorInterrupt, FALLING);
    }
}

ActivityMonitorModule::~ActivityMonitorModule() {
    free(geophoneSensorData.inputData);
    free(geophoneSensorData.outputData);
}

void ActivityMonitorModule::onNotify(uint32_t notification) {
    // Lock collection flag while checking and setting it.
    collectionFlagLock.lock();
    // Ignore alert if data collection ongoing.
    if(!dataCollectionStarted) {
        dataCollectionStarted = true;
        collectionFlagLock.unlock();
        collectData();
    }
}

void ActivityMonitorModule::collectData() {
    // Collect geophone data.
    pthread_t geophoneThread;

    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size = 4 * 1024;
    esp_pthread_set_cfg(&cfg);

    pthread_create(&geophoneThread, NULL, ActivityMonitorModule::geophoneCollectThread, NULL);

    // Collect microphone data.
    //pthread_t microphoneThread;
    //pthread_create(&microphoneThread, NULL, ActivityMonitorModule::microphoneCollectThread, NULL);

    // Wait for data collection to finish.
    pthread_join(geophoneThread, NULL);
    //pthread_join(microphoneThread, NULL);

    // Analyze and send data if event occured.
    analyzeData();

    // Reset collection flag.
    dataCollectionStarted = false;
}

void* ActivityMonitorModule::geophoneCollectThread(void* p) {
    activityMonitorModule->collectGeophoneData();
    return NULL;
}

void* ActivityMonitorModule::microphoneCollectThread(void* p) {
    activityMonitorModule->collectMicrophoneData();
    return NULL;
}

void ActivityMonitorModule::collectGeophoneData() {
    for(int i = 0; i < GEOPHONE_MODULE_SAMPLES; i++) {
        unsigned long microseconds = micros();
        float voltage = 0.0f;
        if(!geophoneSensorData.gs1lfSensor.readVoltage(voltage)) {
            DEBUG_MSG("Error when reading GS1LFSensor voltage!\n");
            dataCollectionStarted = false;
            return;
        }

        geophoneSensorData.inputData[i] = voltage;

        // Sleep for any remaining time between samples.
        unsigned long remainingTime = (1e6 / geophoneSensorData.samplingFrequency) - (micros() - microseconds);
        if(remainingTime > 0) {
            delayMicroseconds(remainingTime);
        }
    }
}

void ActivityMonitorModule::collectMicrophoneData() {
    
}

void ActivityMonitorModule::analyzeData() {
    double totalTime = GEOPHONE_MODULE_SAMPLES / geophoneSensorData.samplingFrequency;

    float maxMagnitude{};
    float fundamentalFreq{};

    fft_config_t *real_fft_plan = fft_init(GEOPHONE_MODULE_SAMPLES, FFT_REAL, FFT_FORWARD, geophoneSensorData.inputData, geophoneSensorData.outputData);
    fft_execute(real_fft_plan);

    for(int i = 1; i < real_fft_plan->size / 2; i++) {
        float mag = sqrt(pow(real_fft_plan->output[2 * i], 2)
            + pow(real_fft_plan->output[2 * i + 1], 2));
        
        float freq = i * 1.0 / totalTime;

        if(mag > maxMagnitude) {
            maxMagnitude = mag;
            fundamentalFreq = freq;
        }
    }

    /*Multiply the magnitude at all other frequencies with (2 / GEOPHONE_MODULE_SAMPLES) to obtain the amplitude at that frequency*/
    float maxAmplitude = maxMagnitude * (2.0 / GEOPHONE_MODULE_SAMPLES);

    //float dcComponent = real_fft_plan->output[0] / GEOPHONE_MODULE_SAMPLES;

    // TODO: Analyze frequencies and send data
    if(
        fundamentalFreq > geophoneSensorData.frequencyRangeThreshold.low &&
        fundamentalFreq < geophoneSensorData.frequencyRangeThreshold.high &&
        maxAmplitude > geophoneSensorData.amplitudeThreshold
    ) {
        DEBUG_MSG("\n(Seismic) Event detected!\n");
        DEBUG_MSG("    Fundamental frequency: %f\n", fundamentalFreq);
        DEBUG_MSG("    Max amplitude: %f\n\n", maxAmplitude);
    }
}
