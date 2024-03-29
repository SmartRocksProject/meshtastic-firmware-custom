#include "MasterLogger.h"

#include <SD.h>
#include <FSCommon.h>

#include <cstdarg>
#include <cstring>
#include <cstdio>
#include <ctime>

#include "GPSStatus.h"
#include "RTC.h"
#include "SPILock.h"

extern meshtastic::GPSStatus *gpsStatus;

FS* MasterLogger::filesystem = &SD;

void MasterLogger::useFallbackFS() {
    filesystem = &FSCom;
}

void MasterLogger::useSDFS() {
    filesystem = &SD;
}

void MasterLogger::writeString(const char* message, ...) {
    // 4K should be enough space for a log entry...
    const int messageMaxLength = 4000;
    char fmtMessage[messageMaxLength];
    memset(fmtMessage, 0, sizeof(fmtMessage));

    va_list arg_ptr;
    va_start(arg_ptr, message);
    vsnprintf(fmtMessage, messageMaxLength - 1, message, arg_ptr);
    va_end(arg_ptr);

    // Open master file as write (defaults to appending)
    {
        concurrency::LockGuard g(spiLock);
        File masterFile = filesystem->open(MASTER_FILE_NAME, "a");
        if(!masterFile) {
            masterFile = filesystem->open(MASTER_FILE_NAME, "w", true);
        }
        if(masterFile) {
            masterFile.println(fmtMessage);
            masterFile.close();
        }
    }
}

void MasterLogger::writeData(LogData& data) {
    // 200 should be enough space for a log entry...
    const int messageMaxLength = 200;
    char fmtMessage[messageMaxLength];
    memset(fmtMessage, 0, sizeof(fmtMessage));

    char timeString[50];
    struct tm ts = *localtime(&data.unixTimeStamp);
    strftime(timeString, sizeof(timeString), "%Y %m %d %H %M %S %Z", &ts); // 23

    GeoCoord coord = GeoCoord(data.gpsData.latitude, data.gpsData.longitude, data.gpsData.altitude);
    // Order: year month day hour minute second timezone detectionType
    //        latDeg latMin latSec latCP lonDeg lonMin lonSec lonCP
    snprintf(
        fmtMessage, messageMaxLength,
        "%u %s %s %d %d %d %c %d %d %d %c",
        data.nodeNum, timeString, data.detectionType == LogData::DETECTION_TYPE_SEISMIC ? "S" : "V",
        coord.getDMSLatDeg(), coord.getDMSLatMin(), coord.getDMSLatSec(), coord.getDMSLatCP(),
        coord.getDMSLonDeg(), coord.getDMSLonMin(), coord.getDMSLonSec(), coord.getDMSLonCP()
    );

    // Open master file as write (defaults to appending)
    {
        concurrency::LockGuard g(spiLock);
        File masterFile = filesystem->open(MASTER_FILE_NAME, "a");
        if(!masterFile) {
            masterFile = filesystem->open(MASTER_FILE_NAME, "w", true);
        }
        if(masterFile) {
            masterFile.println(fmtMessage);
            masterFile.close();
        }
    }
}

MasterLogger::LogData MasterLogger::getLogData(LogData::DetectionType detectionType) {
    MasterLogger::LogData data;
    data.nodeNum = nodeDB.getNodeNum();
    data.gpsData = {
        gpsStatus->getLatitude(),
        gpsStatus->getLongitude(),
        gpsStatus->getAltitude()
    };
    data.unixTimeStamp = getTime();
    data.detectionType = detectionType;
    return data;
}

bool MasterLogger::readLog(String& outLog) {
    {
        concurrency::LockGuard g(spiLock);
        File masterFile = filesystem->open(MASTER_FILE_NAME, "r");
        if(!masterFile) {
            return false;
        }
        outLog = masterFile.readString();

        masterFile.close();
    }
    return true;
}

void MasterLogger::deleteLog() {
    {
        concurrency::LockGuard g(spiLock);
        filesystem->remove(MASTER_FILE_NAME);
    }
}
