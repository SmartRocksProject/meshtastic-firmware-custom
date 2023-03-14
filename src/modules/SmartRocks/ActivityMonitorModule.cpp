// Must be defined first to prevent build errors (conflict with str being defined as a macro by Meshtastic libs)
#include "yashSanghvi_FFT/FFT.h"

#include "ActivityMonitorModule.h"

#include "esp_task_wdt.h"

#include <freertos/task.h>

//#include <arduinoFFT.h>

ActivityMonitorModule* activityMonitorModule;

static void activateMonitor(void* p) {
    while(true) {
        uint32_t ulNotifiedValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(ulNotifiedValue > 0) {
            activityMonitorModule->notify(1, false);
        }
    }
}

static void sensorInterrupt() {
    BaseType_t taskYieldRequired = pdFALSE;

    vTaskNotifyGiveFromISR(activityMonitorModule->runningTaskHandle, &taskYieldRequired);

    portYIELD_FROM_ISR(taskYieldRequired);
}

ActivityMonitorModule::ActivityMonitorModule()
    : concurrency::NotifiedWorkerThread("ActivityMonitorModule")
{
    if(geophoneSensorData.gs1lfSensor.setup(geophoneSensorData.lowThreshold, geophoneSensorData.highThreshold)) {
        geophoneSensorData.inputData = (float*) ps_malloc(sizeof(float) * GEOPHONE_MODULE_SAMPLES);
        geophoneSensorData.outputData = (float*) ps_malloc(sizeof(float) * GEOPHONE_MODULE_SAMPLES);

        xTaskCreate(activateMonitor, "activateMonitor", 1024, NULL, tskIDLE_PRIORITY, &runningTaskHandle);
        
        pinMode(ADS1115_ALERT_PIN, INPUT);
        attachInterrupt(ADS1115_ALERT_PIN, sensorInterrupt, FALLING);
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
    geophoneCollecting.lock();
    xTaskCreate(ActivityMonitorModule::geophoneCollectThread, "geophoneCollectThread", 4 * 1024, NULL, 5, NULL);

    // Collect microphone data.
    microphoneCollecting.lock();
    xTaskCreate(ActivityMonitorModule::microphoneCollectThread, "microphoneCollectThread", 4 * 1024, NULL, 5, NULL);

    // Wait for data collection to finish by waiting for both locks to be unlocked.
    geophoneCollecting.lock();
    microphoneCollecting.lock();
    geophoneCollecting.unlock();
    microphoneCollecting.unlock();

    // Analyze and send data if event occured.
    analyzeData();

    // Reset collection flag.
    dataCollectionStarted = false;
}

void ActivityMonitorModule::geophoneCollectThread(void* p) {
    activityMonitorModule->collectGeophoneData();
    activityMonitorModule->geophoneCollecting.unlock();
}

void ActivityMonitorModule::microphoneCollectThread(void* p) {
    activityMonitorModule->collectMicrophoneData();
    activityMonitorModule->microphoneCollecting.unlock();
}

void ActivityMonitorModule::collectGeophoneData() {
    DEBUG_MSG("Collecting geophone data...\n");
    geophoneSensorData.gs1lfSensor.setContinuousMode(true);
    delay(1); // Give sensor time to switch to continuous mode.
    esp_task_wdt_reset();
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
        long long remainingTime = (1e6 / geophoneSensorData.samplingFrequency) - (long long) (micros() - microseconds);
        if(remainingTime < 0) {
            DEBUG_MSG("Sampling frequency too high!\n");
            dataCollectionStarted = false;
        } else if(remainingTime > 0) {
            delayMicroseconds(remainingTime);
        }
        esp_task_wdt_reset();
    }
    geophoneSensorData.gs1lfSensor.setContinuousMode(false);
    DEBUG_MSG("Finished collecting geophone data.\n");

    vTaskDelete(NULL);
}

void ActivityMonitorModule::collectMicrophoneData() {
    vTaskDelete(NULL);
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

    // Analyze frequencies and send data
    if(
        fundamentalFreq > geophoneSensorData.frequencyRangeThreshold.low &&
        fundamentalFreq < geophoneSensorData.frequencyRangeThreshold.high &&
        maxAmplitude > geophoneSensorData.amplitudeThreshold
    ) {
        DEBUG_MSG("\n(Seismic) Event detected!\n");
        DEBUG_MSG("    Fundamental frequency: %f\n", fundamentalFreq);
        DEBUG_MSG("    Max amplitude: %f\n\n", maxAmplitude);
    } else {
        DEBUG_MSG("\nNo event detected.\n\n");
    }
}
