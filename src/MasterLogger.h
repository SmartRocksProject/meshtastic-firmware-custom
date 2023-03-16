#pragma once


#include "GeoCoord.h"

#include <WString.h>
#include <cstdint>
#include <FS.h>

#define MASTER_FILE_NAME "/Masterfile.txt"

class MasterLogger {
public:
    struct LogData {
        // Construct via GeoCoord(int32_t lat, int32_t lon, int32_t alt)
        // Where lat, lon, and alt are from `extern meshtastic::GPSStatus *gpsStatus` global var.
        // gpsStatus->getLatitude(), gpsStatus->getLongitude(), gpsStatus->GetAltitude()
        GeoCoord gpsData;

        // Retrieve via RTC.h -> getTime() or getValidTime(RTCQuality minQuality)
        time_t unixTimeStamp;
        enum DetectionType {
            DETECTION_TYPE_HUMAN,
            DETECTION_TYPE_VEHICLE
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

    static bool readLog(String& outLog);

    static void useFallbackFS();
    static void useSDFS();
private:
    static FS* filesystem;
};
