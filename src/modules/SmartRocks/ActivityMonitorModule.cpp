// Must be defined first to prevent build errors (conflict with str being defined as a macro by Meshtastic libs)
#include "yashSanghvi_FFT/FFT.h"

#include "ActivityMonitorModule.h"

#include <esp_task_wdt.h>
#include <freertos/task.h>

#include "MasterLogger.h"
#include "MeshService.h"
#include "GPSStatus.h"
#include "RTC.h"

extern meshtastic::GPSStatus *gpsStatus;

static void sensorInterrupt() {
    BaseType_t taskYieldRequired = pdFALSE;

    vTaskNotifyGiveFromISR(activityMonitorModule->runningTaskHandle, &taskYieldRequired);

    portYIELD_FROM_ISR(taskYieldRequired);
}
ActivityMonitorModule* activityMonitorModule;

static void activateMonitor(void* p) {
    pinMode(ADS1115_ALERT_PIN, INPUT);
    attachInterrupt(ADS1115_ALERT_PIN, sensorInterrupt, RISING);
    while(true) {
        uint32_t ulNotifiedValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(ulNotifiedValue > 0) {
            activityMonitorModule->notify(1, false);
        }
    }
    vTaskDelete(NULL);
}

ActivityMonitorModule::ActivityMonitorModule()
    : ProtobufModule("ActivityMonitorModule", meshtastic_PortNum_ACTIVITY_MONITOR_APP, &meshtastic_ActivityMonitorModuleConfig_msg), concurrency::NotifiedWorkerThread("ActivityMonitorModule")
{
    if(geophoneSensorData.gs1lfSensor.setup(geophoneSensorData.lowThreshold, geophoneSensorData.highThreshold)) {
        geophoneSensorData.inputData = (float*) ps_malloc(sizeof(float) * geophoneSensorData.numSamples);
        geophoneSensorData.outputData = (float*) ps_malloc(sizeof(float) * geophoneSensorData.numSamples);
        geophoneInitialized = true;

        xTaskCreate(activateMonitor, "activateMonitor", 1024, NULL, tskIDLE_PRIORITY, &runningTaskHandle);
    }
    if(microphoneSensorData.inmp441Sensor.setup(microphoneSensorData.samplingFrequency, microphoneSensorData.vadBufferLength)) {
        microphoneSensorData.vadBuffer = (int16_t*) ps_malloc(sizeof(int16_t) * microphoneSensorData.vadBufferLength);
        microphoneSensorData.vad_inst = vad_create(VAD_MODE_4);

        microphoneInitialized = true;
    }
    LOG_INFO("Geophone initialized: %d, Microphone initialized: %d\n", geophoneInitialized, microphoneInitialized);
}

ActivityMonitorModule::~ActivityMonitorModule() {
    free(geophoneSensorData.inputData);
    free(geophoneSensorData.outputData);
    free(microphoneSensorData.vadBuffer);
    free(microphoneSensorData.vad_inst);
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
    geophoneSensorData.successfulRead = false;
    microphoneSensorData.successfulRead = false;
    // Collect geophone data.
    if(geophoneInitialized) {
        //geophoneCollecting.lock();
        //xTaskCreate(ActivityMonitorModule::geophoneCollectThread, "geophoneCollectThread", 4 * 1024, NULL, 5, NULL);
        collectGeophoneData();
    }

    // Collect microphone data.
    if(microphoneInitialized) {
        //microphoneCollecting.lock();
        //xTaskCreate(ActivityMonitorModule::microphoneCollectThread, "microphoneCollectThread", 4 * 1024, NULL, 5, NULL);
        collectMicrophoneData();
    }
    
    // Wait for data collection to finish by waiting for both locks to be unlocked.
    /*
    if(geophoneInitialized) {
        geophoneCollecting.lock();
        geophoneCollecting.unlock();
    }
    if(microphoneInitialized) {
        microphoneCollecting.lock();
        microphoneCollecting.unlock();
    }
    */

    analyzeData();
    geophoneSensorData.successfulRead = false;
    microphoneSensorData.successfulRead = false;

    // Reset collection flag.
    dataCollectionStarted = false;
}

void ActivityMonitorModule::geophoneCollectThread(void* p) {
    activityMonitorModule->collectGeophoneData();
    activityMonitorModule->geophoneCollecting.unlock();
    vTaskDelete(NULL);
}

void ActivityMonitorModule::microphoneCollectThread(void* p) {
    activityMonitorModule->collectMicrophoneData();
    activityMonitorModule->microphoneCollecting.unlock();
    vTaskDelete(NULL);
}

void ActivityMonitorModule::collectGeophoneData() {
    LOG_INFO("Collecting geophone data...\n");
    geophoneSensorData.gs1lfSensor.setContinuousMode(true);
    delay(1);
    for(int i = 0; i < geophoneSensorData.numSamples; i++) {
        unsigned long microseconds = micros();
        float voltage = 0.0f;
        if(!geophoneSensorData.gs1lfSensor.readVoltage(voltage)) {
            LOG_INFO("Error when reading GS1LFSensor voltage!\n");
            return;
        }

        geophoneSensorData.inputData[i] = voltage;

        // Sleep for any remaining time between samples.
        long long remainingTime = (1e6 / geophoneSensorData.samplingFrequency) - (long long) (micros() - microseconds);
        if(remainingTime < 0) {
            LOG_INFO("(GS1LF) Sampling frequency too high!\n");
        } else if(remainingTime > 0) {
            delayMicroseconds(remainingTime);
        }
    }
    geophoneSensorData.gs1lfSensor.setContinuousMode(false);
    LOG_INFO("Finished collecting geophone data.\n");
    geophoneSensorData.successfulRead = true;
}

void ActivityMonitorModule::collectMicrophoneData() {
    // Collect microphone data.
    LOG_INFO("Collecting microphone data...\n");

    size_t num_bytes;
    if((num_bytes = microphoneSensorData.inmp441Sensor.readSamples(microphoneSensorData.vadBuffer)) < microphoneSensorData.vadBufferLength) {
        LOG_INFO("Error when reading microphone data! Read %u bytes\n", num_bytes);
        return;
    }

    LOG_INFO("Finished collecting microphone data.\n");
    microphoneSensorData.successfulRead = true;
}

void ActivityMonitorModule::analyzeData() {
    if(geophoneSensorData.successfulRead) {
        analyzeGeophoneData();
    }

    if(microphoneSensorData.successfulRead) {
        analyzeMicrophoneData();
    }
}

void ActivityMonitorModule::analyzeGeophoneData() {
    double totalTime = geophoneSensorData.numSamples / geophoneSensorData.samplingFrequency;

    float maxMagnitude{};
    float fundamentalFreq{};

    fft_config_t *real_fft_plan = fft_init(geophoneSensorData.numSamples, FFT_REAL, FFT_FORWARD, geophoneSensorData.inputData, geophoneSensorData.outputData);
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

    /*Multiply the magnitude at all other frequencies with (2 / geophoneSensorData.numSamples) to obtain the amplitude at that frequency*/
    float maxAmplitude = maxMagnitude * (2.0 / geophoneSensorData.numSamples);

    //float dcComponent = real_fft_plan->output[0] / geophoneSensorData.numSamples;

    // Analyze frequencies and send data
    if(
        fundamentalFreq > geophoneSensorData.frequencyRangeThreshold.low &&
        fundamentalFreq < geophoneSensorData.frequencyRangeThreshold.high &&
        maxAmplitude > geophoneSensorData.amplitudeThreshold
    ) {
        LOG_INFO("\n(Seismic) Event detected!\n");
        LOG_INFO("    Fundamental frequency: %f\n", fundamentalFreq);
        LOG_INFO("    Max amplitude: %f\n\n", maxAmplitude);
        
    #ifdef ACTIVITY_LOG_TO_FILE
        MasterLogger::LogData data = MasterLogger::getLogData(MasterLogger::LogData::DETECTION_TYPE_SEISMIC);
        MasterLogger::writeData(data);
        sendActivityMonitorData(data);
    #endif
    } else {
        LOG_INFO("(Seismic) No event detected.\n\n");
    }
}

void ActivityMonitorModule::analyzeMicrophoneData() {
    vad_state_t vadState = vad_process(microphoneSensorData.vad_inst, microphoneSensorData.vadBuffer, microphoneSensorData.vadSampleRate, microphoneSensorData.vadFrameLengthMs);
    if(vadState == VAD_SPEECH) {
        LOG_INFO("\n(Vocal) Event detected!\n\n");
    #ifdef ACTIVITY_LOG_TO_FILE
        MasterLogger::LogData data = MasterLogger::getLogData(MasterLogger::LogData::DETECTION_TYPE_VOICE);
        MasterLogger::writeData(data);
        sendActivityMonitorData(data);
    #endif
    } else {
        LOG_INFO("(Vocal) No event detected.\n\n");
    }
}

void ActivityMonitorModule::sendActivityMonitorData(MasterLogger::LogData& data, NodeNum dest, bool wantReplies) {
    meshtastic_ActivityMonitorModuleConfig am;
    am.nodeNum = data.nodeNum;
    am.has_gpsData = true;
    am.gpsData = {
        .latitude = data.gpsData.latitude,
        .longitude = data.gpsData.longitude,
        .altitude = data.gpsData.altitude,
    };
    am.unixTimeStamp = (int64_t) data.unixTimeStamp;
    am.detectionType = data.detectionType == MasterLogger::LogData::DETECTION_TYPE_SEISMIC
        ? meshtastic_ActivityMonitorModuleConfig_DetectionType_DETECTION_TYPE_SEISMIC
        : meshtastic_ActivityMonitorModuleConfig_DetectionType_DETECTION_TYPE_VOICE;
    
    meshtastic_MeshPacket* p = allocDataProtobuf(am);
    p->to = dest;
    p->decoded.want_response = wantReplies;
    p->priority = meshtastic_MeshPacket_Priority_MAX;

    LOG_DEBUG("Sending activity monitor data to network\n");
    service.sendToMesh(p);
}

bool ActivityMonitorModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_ActivityMonitorModuleConfig *pptr) {
#ifdef ACTIVITY_LOG_TO_FILE
    auto p = *pptr;

    MasterLogger::LogData data;
    data.nodeNum = p.nodeNum;
    data.gpsData.latitude = p.gpsData.latitude;
    data.gpsData.longitude = p.gpsData.longitude;
    data.gpsData.altitude = p.gpsData.altitude;
    data.unixTimeStamp = (time_t) p.unixTimeStamp;
    data.detectionType = p.detectionType == meshtastic_ActivityMonitorModuleConfig_DetectionType_DETECTION_TYPE_SEISMIC
        ? MasterLogger::LogData::DETECTION_TYPE_SEISMIC
        : MasterLogger::LogData::DETECTION_TYPE_VOICE;
    
    LOG_DEBUG("Received activity monitor data from node: %d\n", data.nodeNum);
    MasterLogger::writeData(data);
#endif
    if(mp.decoded.want_response) {
        // Send a reply
        meshtastic_MeshPacket* reply = allocReply();
        reply->to = mp.from;
        reply->decoded.want_response = false;
        reply->priority = meshtastic_MeshPacket_Priority_MAX;
        service.sendToMesh(reply);
    }

    return false; // Let others look at this message also if they want
}

meshtastic_MeshPacket* ActivityMonitorModule::allocReply() {
    auto reply = allocDataPacket();
    return reply;
}
