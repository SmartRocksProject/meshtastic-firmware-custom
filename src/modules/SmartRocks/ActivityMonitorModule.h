#pragma once

#include <esp_pthread.h>
#include <esp_vad.h>

#include "concurrency/NotifiedWorkerThread.h"
#include "concurrency/Lock.h"
#include "Sensor/GS1LFSensor.h"
#include "Sensor/INMP441Sensor.h"
#include "ProtobufModule.h"
#include "mesh/generated/meshtastic/activitymonitor.pb.h"
#include "MasterLogger.h"

class ActivityMonitorModule : public ProtobufModule<meshtastic_ActivityMonitorModuleConfig>, public concurrency::NotifiedWorkerThread {
public:
    ActivityMonitorModule();
    ~ActivityMonitorModule();

    virtual void onNotify(uint32_t notification) override;

    TaskHandle_t runningTaskHandle{};
protected:
    /** Called to handle a particular incoming message

    @return true if you've guaranteed you've handled this message and no other handlers should be considered for it
    */
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_ActivityMonitorModuleConfig *pptr) override;

    /** Messages can be received that have the want_response bit set.  If set, this callback will be invoked
     * so that subclasses can (optionally) send a response back to the original sender.  */
    virtual meshtastic_MeshPacket* allocReply() override;
private:
    void collectData();
    void analyzeData();

    void collectGeophoneData();
    void collectMicrophoneData();

    void analyzeGeophoneData();
    void analyzeMicrophoneData();

    static void geophoneCollectThread(void* p);
    static void microphoneCollectThread(void* p);

    void sendActivityMonitorData(MasterLogger::LogData& data, NodeNum dest = NODENUM_BROADCAST, bool wantReplies = false);
private:
    concurrency::Lock geophoneCollecting{};
    concurrency::Lock microphoneCollecting{};
    concurrency::Lock collectionFlagLock{};
    bool dataCollectionStarted{};

    struct GeophoneSensorData {
        GS1LFSensor gs1lfSensor;

        float* inputData;
        float* outputData;

        bool successfulRead{false};
        enum { numSamples = 4096 };
        const double samplingFrequency{860.0};
        const double lowThreshold{0.3};
        const double highThreshold{1.0};
        const double amplitudeThreshold{0.25};
        const struct freqRange {
            double low;
            double high;
        } frequencyRangeThreshold{.low = 2.0, .high = 80.0};
    } geophoneSensorData;
    bool geophoneInitialized{};

    struct MicrophoneSensorData {
        INMP441Sensor inmp441Sensor;
        
        enum { 
            vadSampleRate = 16000,
            vadFrameLengthMs = 1000,
            vadBufferLength = (vadFrameLengthMs * vadSampleRate / 1000)
        };
        vad_handle_t vad_inst;
        int16_t* vadBuffer;
        //uint8_t* inputData;

        bool successfulRead{false};
        const uint32_t samplingFrequency{vadSampleRate};
        /*
        const double amplitudeThreshold{1.0};
        const struct freqRange {
            double low;
            double high;
        } frequencyRangeThreshold{.low = 0.0, .high = 100.0};
        */
    } microphoneSensorData;
    bool microphoneInitialized{};
};

extern ActivityMonitorModule* activityMonitorModule;
