#pragma once

#include "GeoCoord.h"
#include "NodeDB.h"

#include <WString.h>
#include <cstdint>
#include <FS.h>

#define MASTER_FILE_NAME "/Masterfile.txt"

class MasterLogger {
public:
    struct LogData {
        // NodeDB.h -> nodeDB.getNodeNum()
        NodeNum nodeNum;

        // lat, lon, and alt are from `extern meshtastic::GPSStatus *gpsStatus` global var.
        // gpsStatus->getLatitude(), gpsStatus->getLongitude(), gpsStatus->GetAltitude()
        struct GPS {
            int32_t latitude;
            int32_t longitude;
            int32_t altitude;
        } gpsData;

        // Retrieve via RTC.h -> getTime() or getValidTime(RTCQuality minQuality)
        time_t unixTimeStamp;
        enum DetectionType {
            DETECTION_TYPE_SEISMIC,
            DETECTION_TYPE_VOICE
        } detectionType;
    };

    /**
     * Write a generic string message to the Masterfile.
     * Takes in variadic arguments for formatting
     */
    static void writeString(const char* message, ...);

    /**
     * Write LogData to the Masterfile as a string.
     */
    static void writeData(LogData& data);

    static LogData getLogData(LogData::DetectionType detectionType);

    static bool readLog(String& outLog);

    static void useFallbackFS();
    static void useSDFS();
private:
    static FS* filesystem;
};
